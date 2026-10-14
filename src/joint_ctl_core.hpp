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
//   3. ALLOWLIST. Only the joints in `allowed` accept a target (J2: the two elbows). The rest
//      stay held where they were taken.
//   4. LIMITS. A target is clamped to the URDF range minus `margin`, AND to `max_excursion` from
//      where the joint was when control was taken: a bad number cannot swing an arm far.
//   5. SPEED. The commanded position walks towards the target at most max_vel rad/s; the
//      browser only names targets, it never sets the pace.
//   6. DEAD-MAN. No message for deadman_ms: the target is frozen where the arm is. None for
//      release_ms: control is released.
//   7. RELEASE IS A RAMP TOO. The weight falls 1 -> 0 over ramp_s while holding position, so
//      PC1 takes the arm back to its own pose smoothly. `release`, EOF on stdin (the relay died),
//      the dead-man, a lost state and rule 1 all end this way. Then the process is finished.
//
// Protocol on stdin (one command per line, whitespace-separated, like sender_core.hpp):
//   take  |  target <joint> <rad>  |  keepalive  |  status  |  release
// Every reply is one line, "ok ..." or "err ...", ending in " state=<s> w=<weight>".
// Asynchronous events come as "ev <what>" lines.
#pragma once

#include <algorithm>
#include <cmath>
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
};

// Body indices 12..28, ranges from g1_29dof_lock_waist_with_hand_rev_1_0.urdf (Unitree's
// g1_description). Waist roll and pitch have no motor on this robot (lock-waist variant,
// mode_machine 6): they are held like every other joint and never accept a target.
constexpr Joint JOINTS[N] = {
    {"waist_yaw", -2.618f, 2.618f},
    {"waist_roll", -0.52f, 0.52f},
    {"waist_pitch", -0.52f, 0.52f},
    {"left_shoulder_pitch", -3.0892f, 2.6704f},
    {"left_shoulder_roll", -1.5882f, 2.2515f},
    {"left_shoulder_yaw", -2.618f, 2.618f},
    {"left_elbow", -1.0472f, 2.0944f},
    {"left_wrist_roll", -1.97222f, 1.97222f},
    {"left_wrist_pitch", -1.61443f, 1.61443f},
    {"left_wrist_yaw", -1.61443f, 1.61443f},
    {"right_shoulder_pitch", -3.0892f, 2.6704f},
    {"right_shoulder_roll", -2.2515f, 1.5882f},
    {"right_shoulder_yaw", -2.618f, 2.618f},
    {"right_elbow", -1.0472f, 2.0944f},
    {"right_wrist_roll", -1.97222f, 1.97222f},
    {"right_wrist_pitch", -1.61443f, 1.61443f},
    {"right_wrist_yaw", -1.61443f, 1.61443f},
};
constexpr const char* LOCKED[] = {"waist_roll", "waist_pitch"};

inline int index_of(const std::string& name) {
    for (int i = 0; i < N; ++i)
        if (name == JOINTS[i].name) return i;
    return -1;
}

struct Config {
    std::set<std::string> allowed{"left_elbow", "right_elbow"};
    std::set<int> fsm_ok{500, 501};   // Walk with the waist locked / free (g1_command_sender)
    float max_vel = 0.5f;             // rad/s, Unitree's example
    float ramp_s = 2.0f;              // weight 0 -> 1 and back
    float max_excursion = 0.6f;       // rad (~34°) from where the joint was taken
    float margin = 0.05f;             // rad inside the URDF range
    float kp = 60.0f, kd = 1.5f;      // Unitree's example
    long deadman_ms = 1000;
    long release_ms = 5000;
    long state_max_age_ms = 100;      // lowstate older than this at `take` = refused
    long state_lost_ms = 500;         // ... and this long while in control = released
    long fsm_lost_ms = 2000;
};

// What the robot reports, as the caller last saw it.
struct Inputs {
    long state_age_ms = -1;           // -1 = no lowstate yet
    float q[NBODY] = {};
    int fsm = -1;                     // -1 = unknown
    long fsm_age_ms = -1;             // since the last successful FSM read; -1 = never
};

// What to publish this tick. publish=false: send nothing at all.
struct Output {
    bool publish = false;
    float weight = 0.0f;
    float q[N] = {};
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
            float lo = std::max(j.lo + cfg_.margin, origin_[i] - cfg_.max_excursion);
            float hi = std::min(j.hi - cfg_.margin, origin_[i] + cfg_.max_excursion);
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
            } else if (quiet > cfg_.release_ms) {
                begin_release("deadman");
            } else if (quiet > cfg_.deadman_ms && !frozen_) {
                for (int i = 0; i < N; ++i) goal_[i] = cmd_[i];
                frozen_ = true;
                ev.push_back("deadman_freeze");
            }
        }
        for (auto& e : pending_) ev.push_back(e);
        pending_.clear();

        // Rule 5: walk towards the goal at max_vel, whatever the state (holding = goal is cmd).
        const float step = cfg_.max_vel * dt;
        for (int i = 0; i < N; ++i) {
            const float d = goal_[i] - cmd_[i];
            cmd_[i] += std::max(-step, std::min(step, d));
        }

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
    float cmd_[N] = {}, goal_[N] = {}, origin_[N] = {};
    long long last_msg_ms_ = 0;
    bool frozen_ = false;
    bool final_sent_ = false;
    std::string release_reason_;
    std::vector<std::string> pending_;

    static bool is_locked(const std::string& name) {
        for (const char* l : LOCKED)
            if (name == l) return true;
        return false;
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
        }
        weight_ = 0.0f;
        state_ = State::Up;
        touch(now_ms);
        return reply("ok take");
    }

    void begin_release(const std::string& why) {
        if (state_ == State::Down || state_ == State::Done) return;
        for (int i = 0; i < N; ++i) goal_[i] = cmd_[i];   // hold, do not finish a move
        release_reason_ = why;
        state_ = State::Down;
        pending_.push_back("releasing " + why);
    }

    void fill(Output& out, float w) const {
        out.publish = true;
        out.weight = w;
        out.kp = cfg_.kp;
        out.kd = cfg_.kd;
        for (int i = 0; i < N; ++i) out.q[i] = cmd_[i];
    }
};

}  // namespace jointctl
