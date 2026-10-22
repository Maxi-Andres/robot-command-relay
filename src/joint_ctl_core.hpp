// joint_ctl_core.hpp — the safety envelope of g1_joint_sender: moving single arm joints of the
// G1 from the app's Joints page, through the robot's rt/arm_sdk topic.
//
// PURE LOGIC: no DDS, no SDK, no threads, no clock of its own. g1_joint_sender.cpp feeds it the
// robot's state and the time, and publishes what it returns; tests/joint_ctl_test.cpp feeds it
// made-up states and checks every rule below without a robot (and without the SDK, so CI runs
// it). Everything that keeps the arm safe lives HERE, not in the relay and not in the browser —
// the same split as sender_core.hpp for walking.
//
// HOW rt/arm_sdk WORKS (Unitree's g1_arm7_sdk_dds_example.cpp): a LowCmd whose motor_cmd[29].q
// is a WEIGHT from 0 to 1. At 0 the robot's own controller (PC1) moves the arms and waist as
// always; at 1 the 17 arm_sdk joints (waist 12-14, arms 15-28) follow THIS message; between,
// a blend. PC1 keeps balancing the legs throughout. The weight covers all 17 joints at once, so
// taking control of one elbow means holding the other 16 where they are.
//
// THE RULES
//   1. ONLY FROM A STANDING, BALANCED ROBOT. `take` is refused unless the robot's FSM is one of
//      fsm_ok (Walk, standing still — the relay stops any walk first) and its state is fresh.
//      If the FSM leaves that set while in control (damped, fell, someone pressed a mode on the
//      remote), or cannot be read for fsm_lost_ms, control is released.
//   2. NO JUMP AT THE START. On `take` every joint's target is where it IS; the weight then
//      rises 0 -> 1 over ramp_s, so the arm is handed over without moving.
//   3. ALLOWLIST. Only the joints in `allowed` accept a target (since J3, 2026-10-15: every
//      arm joint and the waist yaw). The rest stay held where they were taken.
//   4. LIMITS. A target is clamped to the URDF range minus `margin` — and, when max_excursion
//      is set (> 0), to that far from where the joint was when control was taken. J2 used 0.6
//      rad; J3 turned it off at the operator's request, so the full range is reachable.
//   5. SPEED AND SMOOTHNESS. The commanded position moves towards the target at most max_vel
//      rad/s, and its speed changes by at most max_acc rad/s² — it speeds up and slows down
//      instead of jumping (2026-10-15: at constant speed, a dragged slider's ~10 targets a
//      second became start-stop-start, felt as jerks). It brakes in time to stop ON the target.
//      The browser only names targets, it never sets the pace.
//   6. DEAD-MAN. No message for deadman_ms: the target is frozen where the arm is. None for
//      release_ms: control is released.
//   7. RELEASE IS A RAMP TOO. The weight falls 1 -> 0 over ramp_s while holding position, so
//      PC1 takes the arm back to its own pose smoothly. `release`, EOF on stdin (the relay died),
//      the dead-man, a lost state, rules 1 and 9 all end this way. Then the process is finished.
//   8. NO PUSHING INTO THINGS. The URDF range does not know about the robot's own body: an arm
//      sent through the full range can hit the torso or the other arm, and then the motor
//      pushes at full gain, heating up. A joint whose measured torque stays over stall_frac of
//      its rated effort for stall_ms while it is short of its command gives up: command and
//      target become where it IS, so the force drops at once ("ev stall <joint>"). 30% / 150 ms
//      since 2026-10-15: at 60% / 250 ms a person holding the arm felt it keep pushing (at kp
//      60, 60% of an arm joint's 25 N·m is only reached ~14° behind the command; 30% is ~7°).
//      The torque compared is what is LEFT after holding the arm's own weight
//      (arm_gravity.hpp), since 2026-10-16: lifting the left arm forward with the elbow bent
//      costs ~8.5 N·m of gravity alone, over the 7.5 of 30%, and the guard stopped an arm that
//      was only being raised. With gravity taken out, a push still trips it at the same force.
//   9. HOT MOTOR, HANDS OFF. Any held joint at or over temp_release °C releases control.
//  10. FINGERS (since 2026-10-15). The BrainCo hands take a target per finger, 0 open .. 1000
//      closed, only inside a session (up/active) and only for a hand that answered when control
//      was taken. They start at where each finger IS. The hand moves itself at hand_speed (of
//      1000) and limits its own current, so this only clamps and rate-limits the writes to one
//      per hand_min_ms. ON RELEASE each hand goes back to where it was when control was taken,
//      during the arms' hand-back ramp — the operator's call, 2026-10-15: "give the arms back"
//      means the whole upper body, and a hand left closed after the page let go surprised them.
//  11. HOLD THE ARM'S OWN WEIGHT (gravity feedforward, 2026-10-22). With only the kp spring, an
//      arm sags until the spring pays for its weight: 8.5 N·m on a raised left shoulder at kp 60
//      is ~8° below the target, centimetres at the hand — no grasp works like that. Each joint's
//      message now also carries the torque that holds the arm up at the COMMANDED pose
//      (arm_gravity.hpp, the model rule 8 already trusts), so the spring only corrects the rest.
//      The commanded pose, not the measured one: what the arm should weigh where it is going,
//      with no feedback from its own sag or sensor noise. Scaled by the hand-over weight (rises
//      with `take`, falls with the release, 0 in the last message), capped at ff_max_frac of
//      each joint's rated effort, and zero for a tick whose pose or tilt is not finite.
//      ff_gain 0 turns it off (JOINT_CTL_FEEDFORWARD=0). The first try, 2026-10-16, felt worse —
//      but that night the relay's CPUQuota=25% was freezing the whole process in bursts: the
//      unit with CPUWeight was written but not yet installed on the robot. It is now (checked
//      2026-10-22), so this has never really been measured.
//
// Protocol on stdin (one command per line, whitespace-separated, like sender_core.hpp):
//   take  |  target <joint> <rad>  |  hand <left|right> <finger 0-5> <pos 0-1000>
//   keepalive  |  status  |  release
// Every reply is one line, "ok ..." or "err ...", ending in " state=<s> w=<weight>".
// Asynchronous events come as "ev <what>" lines.
#pragma once

#include <algorithm>
#include <cmath>

#include "arm_gravity.hpp"
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace jointctl {

constexpr int NBODY = 29;     // the SDK's G1 29-DoF order (rt/lowstate motor_state[0..28])
constexpr int FIRST = 12;     // the arm_sdk joints are body indices 12..28
constexpr int N = 17;
constexpr int WEIGHT_INDEX = 29;   // motor_cmd[29].q carries the weight (kNotUsedJoint)

struct Joint {
    const char* name;
    float lo, hi;             // the URDF's range, radians
    float effort;             // the URDF's rated torque, N·m
};

// Body indices 12..28, ranges from g1_29dof_lock_waist_with_hand_rev_1_0.urdf (Unitree's
// g1_description). Waist roll and pitch have no motor on this robot (lock-waist variant,
// mode_machine 6): they are held like every other joint and never accept a target.
constexpr Joint JOINTS[N] = {
    {"waist_yaw", -2.618f, 2.618f, 88.0f},
    {"waist_roll", -0.52f, 0.52f, 35.0f},
    {"waist_pitch", -0.52f, 0.52f, 35.0f},
    {"left_shoulder_pitch", -3.0892f, 2.6704f, 25.0f},
    {"left_shoulder_roll", -1.5882f, 2.2515f, 25.0f},
    {"left_shoulder_yaw", -2.618f, 2.618f, 25.0f},
    {"left_elbow", -1.0472f, 2.0944f, 25.0f},
    {"left_wrist_roll", -1.97222f, 1.97222f, 25.0f},
    {"left_wrist_pitch", -1.61443f, 1.61443f, 5.0f},
    {"left_wrist_yaw", -1.61443f, 1.61443f, 5.0f},
    {"right_shoulder_pitch", -3.0892f, 2.6704f, 25.0f},
    {"right_shoulder_roll", -2.2515f, 1.5882f, 25.0f},
    {"right_shoulder_yaw", -2.618f, 2.618f, 25.0f},
    {"right_elbow", -1.0472f, 2.0944f, 25.0f},
    {"right_wrist_roll", -1.97222f, 1.97222f, 25.0f},
    {"right_wrist_pitch", -1.61443f, 1.61443f, 5.0f},
    {"right_wrist_yaw", -1.61443f, 1.61443f, 5.0f},
};
constexpr const char* LOCKED[] = {"waist_roll", "waist_pitch"};

inline int index_of(const std::string& name) {
    for (int i = 0; i < N; ++i)
        if (name == JOINTS[i].name) return i;
    return -1;
}

inline bool is_locked(const std::string& name) {
    for (const char* l : LOCKED)
        if (name == l) return true;
    return false;
}

// Every joint arm_sdk can move on this robot: all of them but the two locked waist joints.
inline std::set<std::string> all_movable() {
    std::set<std::string> out;
    for (const auto& j : JOINTS)
        if (!is_locked(j.name)) out.insert(j.name);
    return out;
}

struct Config {
    std::set<std::string> allowed = all_movable();
    std::set<int> fsm_ok{500, 501};   // Walk with the waist locked / free (g1_command_sender)
    float max_vel = 0.5f;             // rad/s, Unitree's example
    float max_acc = 2.0f;             // rad/s²: 0 -> max_vel in 0.25 s
    float ramp_s = 2.0f;              // weight 0 -> 1 and back
    float max_excursion = 0.0f;       // rad from where the joint was taken; 0 = no such limit
    float margin = 0.05f;             // rad inside the URDF range
    float kp = 60.0f, kd = 1.5f;      // Unitree's example
    long deadman_ms = 1000;
    long release_ms = 5000;
    long state_max_age_ms = 100;      // lowstate older than this at `take` = refused
    long state_lost_ms = 500;         // ... and this long while in control = released
    long fsm_lost_ms = 2000;
    float stall_frac = 0.3f;          // rule 8: share of the rated effort that counts as pushing
    float stall_gap = 0.05f;          // ... while this far (rad) from its command
    long stall_ms = 150;              // ... for this long
    int temp_release = 75;            // rule 9: °C; the telemetry alarm is 80
    int hand_speed = 300;             // rule 10: of the hand's 1..1000
    long hand_min_ms = 50;            // ... at most one write per hand this often
    float ff_gain = 1.0f;             // rule 11: share of the modelled holding torque sent; 0 = off
    float ff_max_frac = 0.5f;         // ... never more than this share of a joint's rated effort
};

// What the robot reports, as the caller last saw it.
struct Inputs {
    long state_age_ms = -1;           // -1 = no lowstate yet
    float q[NBODY] = {};
    float tau[NBODY] = {};            // estimated torque, N·m
    int temp[NBODY] = {};             // °C, the hotter of the motor's two sensors
    float rpy[3] = {};                // pelvis IMU roll, pitch, yaw (rad): which way is down
    int fsm = -1;                     // -1 = unknown
    long fsm_age_ms = -1;             // since the last successful FSM read; -1 = never
    bool hand_ok[2] = {false, false}; // left, right: answering right now
    int hand_pos[2][6] = {};          // their fingers, 0 open .. 1000 closed
};

constexpr int HAND_FINGERS = 6;
inline int hand_side(const std::string& s) { return s == "left" ? 0 : s == "right" ? 1 : -1; }

// What to publish this tick. publish=false: send nothing at all.
struct Output {
    bool publish = false;
    float weight = 0.0f;
    float q[N] = {};
    float tau[N] = {};                // rule 11: feedforward torque, N·m, already weight-scaled
    float kp = 0.0f, kd = 0.0f;
};

enum class State { Idle, Up, Active, Down, Done };

inline const char* state_name(State s) {
    switch (s) {
        case State::Idle: return "idle";
        case State::Up: return "up";
        case State::Active: return "active";
        case State::Down: return "down";
        case State::Done: return "done";
    }
    return "?";
}

class Controller {
public:
    explicit Controller(Config cfg) : cfg_(std::move(cfg)) {}

    State state() const { return state_; }
    bool finished() const { return state_ == State::Done && final_sent_; }
    float weight() const { return weight_; }
    float commanded(int i) const { return cmd_[i]; }
    float goal(int i) const { return goal_[i]; }

    // One stdin line. `now_ms` is the caller's monotonic clock.
    std::string handle(const std::string& line, long long now_ms, const Inputs& in) {
        std::istringstream is(line);
        std::string verb;
        if (!(is >> verb)) return reply("err empty");
        // Any well-formed message from the relay is a sign of life for the dead-man.
        if (verb == "keepalive") {
            touch(now_ms);
            return reply("ok keepalive");
        }
        if (verb == "status") return reply("ok status");
        if (verb == "take") return take(now_ms, in);
        if (verb == "release") {
            if (state_ == State::Idle) {
                state_ = State::Done;           // nothing was ever published: nothing to undo
                final_sent_ = true;
                return reply("ok release");
            }
            begin_release("requested");
            return reply("ok release");
        }
        if (verb == "hand") {
            std::string side;
            int finger = -1;
            float pos = 0;
            if (!(is >> side >> finger >> pos) || !std::isfinite(pos))
                return reply("err hand needs side finger pos");
            const int h = hand_side(side);
            if (h < 0 || finger < 0 || finger >= HAND_FINGERS)
                return reply("err no such finger");
            if (state_ != State::Up && state_ != State::Active)
                return reply("err not in control");
            if (!hand_taken_[h]) return reply("err " + side + " hand not available");
            touch(now_ms);
            hand_goal_[h][finger] = std::min(1000, std::max(0, static_cast<int>(std::lround(pos))));
            hand_dirty_[h] = true;
            return reply("ok hand " + side + " " + std::to_string(finger) + " " +
                         std::to_string(hand_goal_[h][finger]));
        }
        if (verb == "target") {
            std::string name;
            float v = 0;
            if (!(is >> name >> v) || !std::isfinite(v)) return reply("err target needs joint rad");
            const int i = index_of(name);
            if (i < 0 || !cfg_.allowed.count(name) || is_locked(name))
                return reply("err joint not allowed: " + name);
            if (state_ != State::Up && state_ != State::Active)
                return reply("err not in control");
            touch(now_ms);
            const Joint& j = JOINTS[i];
            float lo = j.lo + cfg_.margin, hi = j.hi - cfg_.margin;
            if (cfg_.max_excursion > 0) {
                lo = std::max(lo, origin_[i] - cfg_.max_excursion);
                hi = std::min(hi, origin_[i] + cfg_.max_excursion);
            }
            goal_[i] = std::min(hi, std::max(lo, v));
            std::ostringstream os;
            os << "ok target " << name << ' ' << fmt(goal_[i]);
            return reply(os.str());
        }
        return reply("err unknown verb");
    }

    // stdin closed: the relay is gone. Never leave the arm held by a dead caller.
    void on_eof() {
        if (state_ == State::Idle) {
            state_ = State::Done;
            final_sent_ = true;
        } else {
            begin_release("eof");
        }
    }

    // One control tick of `dt` seconds. Events (rule changes) are appended to `ev`.
    // Rule 10: the six targets to write to hand `h` now, if any are due. The caller writes them
    // and calls hand_written() only once the hand acknowledged, so a lost write is retried.
    bool hand_due(int h, long long now_ms, int* pos, int* speed) const {
        if (h < 0 || h > 1 || !hand_taken_[h] || !hand_dirty_[h]) return false;
        // Down too: that is when a released hand is sent back to where it started.
        if (state_ != State::Up && state_ != State::Active && state_ != State::Down) return false;
        if (now_ms - hand_sent_ms_[h] < cfg_.hand_min_ms) return false;
        for (int i = 0; i < HAND_FINGERS; ++i) {
            pos[i] = hand_goal_[h][i];
            speed[i] = cfg_.hand_speed;
        }
        return true;
    }
    void hand_written(int h, long long now_ms, const int* pos) {
        hand_sent_ms_[h] = now_ms;
        bool same = true;
        for (int i = 0; i < HAND_FINGERS; ++i) same = same && pos[i] == hand_goal_[h][i];
        if (same) hand_dirty_[h] = false;            // a newer goal arrived meanwhile: keep it
    }
    bool hand_taken(int h) const { return h >= 0 && h < 2 && hand_taken_[h]; }
    int hand_goal(int h, int f) const { return hand_goal_[h][f]; }

    Output tick(long long now_ms, float dt, const Inputs& in, std::vector<std::string>& ev) {
        for (auto& e : pending_) ev.push_back(e);
        pending_.clear();
        Output out;
        if (state_ == State::Idle) return out;
        if (state_ == State::Done) {
            // One last weight-0 message, so PC1 owns the arms even if the ramp's last tick was
            // lost; then nothing more is ever published.
            if (!final_sent_) {
                final_sent_ = true;
                fill(out, 0.0f);
            }
            return out;
        }

        if (state_ == State::Up || state_ == State::Active) {
            const long quiet = static_cast<long>(now_ms - last_msg_ms_);
            if (in.state_age_ms < 0 || in.state_age_ms > cfg_.state_lost_ms) {
                begin_release("state_lost");
            } else if (in.fsm >= 0 && !cfg_.fsm_ok.count(in.fsm)) {
                begin_release("fsm_" + std::to_string(in.fsm));
            } else if (in.fsm_age_ms < 0 || in.fsm_age_ms > cfg_.fsm_lost_ms) {
                begin_release("fsm_lost");
            } else if (const char* hot = hot_joint(in)) {
                begin_release(std::string("hot_") + hot);
            } else if (quiet > cfg_.release_ms) {
                begin_release("deadman");
            } else if (quiet > cfg_.deadman_ms && !frozen_) {
                for (int i = 0; i < N; ++i) goal_[i] = stopping_point(i);
                frozen_ = true;
                ev.push_back("deadman_freeze");
            }
        }
        for (auto& e : pending_) ev.push_back(e);
        pending_.clear();

        // Rule 8, on every state that publishes: a joint pushing against something gives up.
        // What counts is the torque beyond holding the arm up (the waist yaw holds no weight).
        float holding[NBODY] = {};
        arm_gravity(in.q, in.rpy[0], in.rpy[1], holding);
        for (int i = 0; i < N; ++i) {
            if (is_locked(JOINTS[i].name)) continue;
            const float q = in.q[FIRST + i];
            const bool pushing =
                std::fabs(in.tau[FIRST + i] - holding[FIRST + i]) >
                    cfg_.stall_frac * JOINTS[i].effort &&
                std::fabs(cmd_[i] - q) > cfg_.stall_gap;
            if (!pushing) {
                stall_since_[i] = -1;
            } else if (stall_since_[i] < 0) {
                stall_since_[i] = now_ms;
            } else if (now_ms - stall_since_[i] >= cfg_.stall_ms && std::isfinite(q)) {
                cmd_[i] = goal_[i] = q;
                vel_[i] = 0.0f;
                stall_since_[i] = -1;
                ev.push_back(std::string("stall ") + JOINTS[i].name);
            }
        }

        // Rule 5: towards the goal at most max_vel, speed changing at most max_acc, braking so
        // as to stop on the goal (v = sqrt(2·a·d) is the fastest speed that still can).
        for (int i = 0; i < N; ++i) step_joint(i, dt);

        // Rule 11: the torque that holds the arm up where it is commanded to be.
        feedforward(in);

        const float dw = cfg_.ramp_s > 0 ? dt / cfg_.ramp_s : 1.0f;
        if (state_ == State::Up) {
            weight_ = std::min(1.0f, weight_ + dw);
            if (weight_ >= 1.0f) {
                state_ = State::Active;
                ev.push_back("state active");
            }
        } else if (state_ == State::Down) {
            weight_ = std::max(0.0f, weight_ - dw);
            if (weight_ <= 0.0f) {
                state_ = State::Done;
                final_sent_ = true;             // this tick IS the weight-0 message
                ev.push_back("released " + release_reason_);
            }
        }
        fill(out, weight_);
        return out;
    }

private:
    Config cfg_;
    State state_ = State::Idle;
    float weight_ = 0.0f;
    float cmd_[N] = {}, goal_[N] = {}, origin_[N] = {}, vel_[N] = {};
    float ff_[N] = {};                // rule 11, before the weight
    long long last_msg_ms_ = 0;
    long long stall_since_[N] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,
                                 -1, -1, -1, -1, -1, -1, -1, -1};
    bool frozen_ = false;
    bool hand_taken_[2] = {false, false};
    bool hand_dirty_[2] = {false, false};
    int hand_goal_[2][HAND_FINGERS] = {};
    int hand_origin_[2][HAND_FINGERS] = {};
    long long hand_sent_ms_[2] = {-1000000, -1000000};
    bool final_sent_ = false;
    std::string release_reason_;
    std::vector<std::string> pending_;

    // Where joint i comes to rest if it brakes now at max_acc.
    float stopping_point(int i) const {
        const float a = cfg_.max_acc > 0 ? cfg_.max_acc : 1e6f;
        return cmd_[i] + vel_[i] * std::fabs(vel_[i]) / (2.0f * a);
    }

    void step_joint(int i, float dt) {
        const float d = goal_[i] - cmd_[i];
        const float a = cfg_.max_acc > 0 ? cfg_.max_acc : 1e6f;
        // The fastest speed that still stops on the goal when braking at `a` in steps of dt:
        // v²/2a + v·dt/2 = |d|. (The continuous v = sqrt(2a·d) arrives with speed left over,
        // and dropping it in one tick is exactly the jolt this rule is here to avoid.)
        const float h = a * dt / 2.0f;
        const float brake = -h + std::sqrt(h * h + 2.0f * a * std::fabs(d));
        const float want = (d >= 0 ? 1.0f : -1.0f) * std::min(cfg_.max_vel, brake);
        const float dv = std::max(-a * dt, std::min(a * dt, want - vel_[i]));
        vel_[i] += dv;
        const float step = vel_[i] * dt;
        // Arrived (or would pass it this tick): sit exactly on the goal, at rest.
        if (std::fabs(d) < 1e-5f || (step * d > 0 && std::fabs(step) >= std::fabs(d))) {
            cmd_[i] = goal_[i];
            vel_[i] = 0.0f;
        } else {
            cmd_[i] += step;
        }
    }

    void feedforward(const Inputs& in) {
        for (int i = 0; i < N; ++i) ff_[i] = 0.0f;
        if (!(cfg_.ff_gain > 0.0f)) return;
        if (!std::isfinite(in.rpy[0]) || !std::isfinite(in.rpy[1])) return;
        float pose[NBODY];
        for (int i = 0; i < NBODY; ++i) pose[i] = in.q[i];
        for (int i = 0; i < N; ++i) pose[FIRST + i] = cmd_[i];
        for (float v : pose)
            if (!std::isfinite(v)) return;
        float hold[NBODY] = {};
        arm_gravity(pose, in.rpy[0], in.rpy[1], hold);    // writes the arms only, not the waist
        for (int i = 0; i < N; ++i) {
            if (is_locked(JOINTS[i].name)) continue;
            const float cap = cfg_.ff_max_frac * JOINTS[i].effort;
            ff_[i] = std::min(cap, std::max(-cap, cfg_.ff_gain * hold[FIRST + i]));
        }
    }

    const char* hot_joint(const Inputs& in) const {
        for (int i = 0; i < N; ++i)
            if (!is_locked(JOINTS[i].name) && in.temp[FIRST + i] >= cfg_.temp_release)
                return JOINTS[i].name;
        return nullptr;
    }

    static std::string fmt(float v) {
        std::ostringstream os;
        os.setf(std::ios::fixed);
        os.precision(4);
        os << v;
        return os.str();
    }

    std::string reply(const std::string& head) const {
        std::ostringstream os;
        os << head << " state=" << state_name(state_) << " w=";
        os.setf(std::ios::fixed);
        os.precision(2);
        os << weight_;
        return os.str();
    }

    void touch(long long now_ms) {
        last_msg_ms_ = now_ms;
        frozen_ = false;
    }

    std::string take(long long now_ms, const Inputs& in) {
        if (state_ != State::Idle) return reply("err already taken");
        if (in.state_age_ms < 0 || in.state_age_ms > cfg_.state_max_age_ms)
            return reply("err no fresh lowstate");
        if (in.fsm_age_ms < 0 || in.fsm_age_ms > cfg_.fsm_lost_ms)
            return reply("err fsm unknown");
        if (!cfg_.fsm_ok.count(in.fsm))
            return reply("err fsm " + std::to_string(in.fsm) + " not allowed: stand in Walk");
        for (int i = 0; i < N; ++i) {
            const float q = in.q[FIRST + i];
            if (!std::isfinite(q)) return reply("err bad joint reading");
            origin_[i] = cmd_[i] = goal_[i] = q;
            vel_[i] = 0.0f;
        }
        // Rule 10: a hand is taken only if it answered just now, and its targets start at
        // where its fingers are — taking control never moves a finger.
        for (int h = 0; h < 2; ++h) {
            hand_taken_[h] = in.hand_ok[h];
            hand_dirty_[h] = false;
            for (int f = 0; f < HAND_FINGERS; ++f)
                hand_origin_[h][f] = hand_goal_[h][f] =
                    std::min(1000, std::max(0, in.hand_pos[h][f]));
        }
        weight_ = 0.0f;
        state_ = State::Up;
        touch(now_ms);
        return reply("ok take");
    }

    void begin_release(const std::string& why) {
        if (state_ == State::Down || state_ == State::Done) return;
        for (int i = 0; i < N; ++i) goal_[i] = stopping_point(i);   // hold: brake, do not finish
        // Rule 10: each hand back to where it was when control was taken.
        for (int h = 0; h < 2; ++h) {
            if (!hand_taken_[h]) continue;
            bool moved = false;
            for (int f = 0; f < HAND_FINGERS; ++f) {
                moved = moved || hand_goal_[h][f] != hand_origin_[h][f];
                hand_goal_[h][f] = hand_origin_[h][f];
            }
            hand_dirty_[h] = hand_dirty_[h] || moved;
        }
        release_reason_ = why;
        state_ = State::Down;
        pending_.push_back("releasing " + why);
    }

    void fill(Output& out, float w) const {
        out.publish = true;
        out.weight = w;
        out.kp = cfg_.kp;
        out.kd = cfg_.kd;
        for (int i = 0; i < N; ++i) {
            out.q[i] = cmd_[i];
            out.tau[i] = ff_[i] * w;
        }
    }
};

}  // namespace jointctl
