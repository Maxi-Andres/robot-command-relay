// Drives src/joint_ctl_core.hpp — the rules for moving the G1's arm joints from the Joints page
// — with made-up robot states and a made-up clock. No SDK, no DDS, no robot: plain g++, so CI
// runs it (tests/test_joint_ctl_core.py builds and runs this file).
//
// One check per rule of the header. Prints "ok <what>" / "FAIL <what>" and exits non-zero on any
// failure.
#include "../src/joint_ctl_core.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace jointctl;

namespace {

int failures = 0;

void check(bool cond, const char* what) {
    std::printf("%s %s\n", cond ? "ok  " : "FAIL", what);
    if (!cond) ++failures;
}

bool starts(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

constexpr int L_ELBOW = 18;           // body index
constexpr int L_ELBOW_ARM = L_ELBOW - FIRST;

Inputs standing() {
    Inputs in;
    in.state_age_ms = 2;
    for (int i = 0; i < NBODY; ++i) in.q[i] = 0.01f * i;
    in.q[L_ELBOW] = 0.9f;
    in.fsm = 500;
    in.fsm_age_ms = 100;
    return in;
}

// Runs `seconds` of 50 Hz ticks from `t` (advanced), collecting events and the last output.
Output run(Controller& c, long long& t, float seconds, const Inputs& in,
           std::vector<std::string>& ev, bool keepalive = true) {
    Output last;
    const int n = static_cast<int>(seconds / 0.02f + 0.5f);
    for (int k = 0; k < n; ++k) {
        t += 20;
        if (keepalive && k % 10 == 0) c.handle("keepalive", t, in);
        last = c.tick(t, 0.02f, in, ev);
    }
    return last;
}

bool has(const std::vector<std::string>& ev, const char* p) {
    for (const auto& e : ev)
        if (starts(e, p)) return true;
    return false;
}

}  // namespace

int main() {
    // Rule 1: only from a standing, balanced robot.
    {
        Controller c{Config{}};
        Inputs in = standing();
        in.fsm = 1;                                   // damped
        check(starts(c.handle("take", 0, in), "err fsm 1 not allowed"), "take refused when damped");
        in = standing();
        in.state_age_ms = 400;
        check(starts(c.handle("take", 0, in), "err no fresh lowstate"), "take refused on stale state");
        in = standing();
        in.fsm_age_ms = -1;
        check(starts(c.handle("take", 0, in), "err fsm unknown"), "take refused with FSM unknown");
        std::vector<std::string> ev;
        long long t = 0;
        check(!run(c, t, 0.5f, standing(), ev).publish, "nothing published before a take");
    }
    // Rule 2: no jump at the start — targets are where the joints are, weight ramps.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        check(starts(c.handle("take", t, in), "ok take state=up"), "take accepted when standing");
        std::vector<std::string> ev;
        Output o = run(c, t, 1.0f, in, ev);
        check(o.publish && o.weight > 0.45f && o.weight < 0.55f, "weight halfway after 1 s of 2");
        check(o.q[L_ELBOW_ARM] == 0.9f && o.q[0] == 0.12f, "every joint held where it was");
        o = run(c, t, 1.1f, in, ev);
        check(o.weight == 1.0f && c.state() == State::Active && has(ev, "state active"),
              "weight 1 and active after the ramp");
        check(o.kp == 60.0f && o.kd == 1.5f, "Unitree's example gains");
    }
    // Rules 3 and 4: allowlist and limits.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        check(starts(c.handle("target left_elbow 1.0", t, in), "err not in control"),
              "target refused before take");
        c.handle("take", t, in);
        check(starts(c.handle("target waist_pitch 0.1", t, in), "err joint not allowed"),
              "locked waist joint refused");
        check(starts(c.handle("target left_knee 0.1", t, in), "err joint not allowed"),
              "a leg is not an arm_sdk joint");
        check(starts(c.handle("target left_elbow nan", t, in), "err target needs"), "NaN refused");
        // Defaults since J3: every arm joint and the waist yaw, over the full URDF range.
        check(starts(c.handle("target left_shoulder_pitch 0.5", t, in), "ok target"),
              "every arm joint allowed by default");
        check(starts(c.handle("target waist_yaw 0.2", t, in), "ok target"), "and the waist yaw");
        check(starts(c.handle("target left_elbow 9", t, in), "ok target left_elbow 2.0444"),
              "target clamped to the URDF range minus the margin");
        check(starts(c.handle("target left_elbow -9", t, in), "ok target left_elbow -0.9972"),
              "and on the low side");
        Config narrow;
        narrow.allowed = {"left_elbow"};
        narrow.max_excursion = 0.6f;
        Controller d{narrow};
        d.handle("take", t, in);
        check(starts(d.handle("target left_shoulder_pitch 0.5", t, in), "err joint not allowed"),
              "joint outside a narrowed allowlist refused");
        check(starts(d.handle("target left_elbow 9", t, in), "ok target left_elbow 1.5000"),
              "an excursion limit, when set, clamps to that far from the start");
    }
    // Rule 5: speed and smoothness — capped speed, capped acceleration, no overshoot.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.handle("target left_elbow 1.3", t, in);
        float prev = c.commanded(L_ELBOW_ARM), prev_v = 0.0f, max_v = 0.0f, max_a = 0.0f;
        float peak = prev;
        float at_04 = 0.0f;
        for (int k = 1; k <= 200; ++k) {              // 4 s at 50 Hz
            t += 20;
            if (k % 10 == 0) c.handle("keepalive", t, in);
            c.tick(t, 0.02f, in, ev);
            const float q = c.commanded(L_ELBOW_ARM), v = (q - prev) / 0.02f;
            max_v = std::max(max_v, std::fabs(v));
            max_a = std::max(max_a, std::fabs(v - prev_v) / 0.02f);
            peak = std::max(peak, q);
            if (k == 20) at_04 = q - 0.9f;
            prev = q;
            prev_v = v;
        }
        check(max_v <= 0.5f + 1e-3f, "never faster than 0.5 rad/s");
        check(max_a <= 2.0f + 0.3f, "speed never changes faster than ~2 rad/s², arrival included");
        check(at_04 > 0.12f && at_04 < 0.15f, "speeds up gently: ~0.14 rad in the first 0.4 s");
        check(peak <= 1.3f + 1e-4f, "never passes the target");
        check(std::fabs(c.commanded(L_ELBOW_ARM) - 1.3f) < 1e-5f, "and stops exactly on it");
        check(c.commanded(L_ELBOW_ARM + 7) == in.q[L_ELBOW + 7], "the other elbow did not move");
    }
    // Rule 6: dead-man — freeze, then release.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.handle("target left_elbow 1.6", t, in);
        Output o = run(c, t, 1.2f, in, ev, false);    // no keepalive from here on
        check(has(ev, "deadman_freeze"), "frozen after 1 s of silence");
        const float stop_at = c.goal(L_ELBOW_ARM);
        check(stop_at < 1.6f && stop_at > o.q[L_ELBOW_ARM] - 1e-4f,
              "frozen short of the target, braking instead of stopping dead");
        o = run(c, t, 1.0f, in, ev, false);
        check(std::fabs(o.q[L_ELBOW_ARM] - stop_at) < 1e-5f, "and stays where it braked to");
        run(c, t, 3.0f, in, ev, false);
        check(has(ev, "releasing deadman"), "released after 5 s of silence");
    }
    // Rule 7: release ramps down, then publishes nothing more.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        check(starts(c.handle("release", t, in), "ok release state=down"), "release accepted");
        Output o = run(c, t, 1.0f, in, ev);
        check(o.publish && o.weight > 0.45f && o.weight < 0.55f, "weight halfway down after 1 s");
        // Tick by tick to the end: the tick that finishes must itself publish weight 0.
        for (int k = 0; k < 100 && !c.finished(); ++k) {
            t += 20;
            o = c.tick(t, 0.02f, in, ev);
        }
        check(o.publish && o.weight == 0.0f && has(ev, "released requested"),
              "last message has weight 0");
        check(c.finished(), "finished after the ramp");
        o = run(c, t, 0.5f, in, ev);
        check(!o.publish, "nothing published after it");
        check(starts(c.handle("take", t, in), "err already taken"), "one session per process");
    }
    // EOF, a fall, a lost state, a lost FSM: all release.
    {
        struct Case {
            const char* what;
            const char* reason;
            void (*spoil)(Inputs&);
        };
        const Case cases[] = {
            {"FSM left Walk (damped) releases", "releasing fsm_1", [](Inputs& in) { in.fsm = 1; }},
            {"lowstate lost releases", "releasing state_lost",
             [](Inputs& in) { in.state_age_ms = 900; }},
            {"FSM unreadable releases", "releasing fsm_lost",
             [](Inputs& in) { in.fsm_age_ms = 2500; }},
        };
        for (const auto& k : cases) {
            Controller c{Config{}};
            Inputs in = standing();
            long long t = 0;
            c.handle("take", t, in);
            std::vector<std::string> ev;
            run(c, t, 0.5f, in, ev);
            k.spoil(in);
            run(c, t, 0.1f, in, ev);
            check(has(ev, k.reason) && c.state() == State::Down, k.what);
        }
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.on_eof();
        run(c, t, 2.1f, in, ev, false);
        check(has(ev, "released eof") && c.finished(), "EOF ramps down and finishes");
        Controller idle{Config{}};
        idle.on_eof();
        Output o = run(idle, t, 0.1f, in, ev, false);
        check(idle.finished() && !o.publish, "EOF before a take publishes nothing");
    }
    // Rule 8: a joint pushing into something gives up, at once and only that joint.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.handle("target left_elbow 1.4", t, in);
        run(c, t, 0.4f, in, ev);                      // commanded ~1.04, measured still 0.9
        in.tau[L_ELBOW] = 8.0f;                       // 32% of its 25 N·m: pushing on something
        Output o = run(c, t, 0.1f, in, ev);
        check(!has(ev, "stall"), "a short push is tolerated");
        o = run(c, t, 0.1f, in, ev);
        check(has(ev, "stall left_elbow"), "a push held for 150 ms is a stall");
        check(o.q[L_ELBOW_ARM] == 0.9f && c.goal(L_ELBOW_ARM) == 0.9f,
              "command and target drop to where the joint is");
        in.tau[L_ELBOW] = 0.0f;
        o = run(c, t, 1.0f, in, ev);
        check(o.q[L_ELBOW_ARM] == 0.9f && c.state() == State::Active,
              "it stays there, still in control");
        Controller e{Config{}};
        Inputs held = standing();
        held.tau[L_ELBOW + 7] = 8.0f;                 // holding against gravity, no gap
        long long u = 0;
        e.handle("take", u, held);
        std::vector<std::string> ev2;
        run(e, u, 1.0f, held, ev2);
        check(!has(ev2, "stall"), "torque with no gap to the command is not a stall");
    }
    // Rule 8 counts only the torque beyond holding the arm up (arm_gravity.hpp, 2026-10-16).
    {
        // The model, pinned to the Python one that matched the robot's measured torques: the
        // pose captured from the G1 on 2026-10-16 (left arm raised, measured -6.7 / -3.0 N·m).
        const float captured[NBODY] = {-0.3058f, 0.0005f, 0.0121f, 0.4902f, -0.1889f, 0.0022f,
            -0.2968f, 0.0074f, 0.0005f, 0.4655f, -0.1795f, -0.0322f, 0.0028f, 0.0f, 0.0f,
            -0.6985f, 0.1907f, -0.0437f, 0.161f, 0.1274f, 0.0809f, -0.0272f,
            0.2796f, -0.1918f, 0.0445f, 0.9953f, -0.1253f, 0.0469f, 0.0192f};
        float g[NBODY] = {};
        arm_gravity(captured, 0.004f, 0.0479f, g);
        check(std::fabs(g[15] + 5.5359f) < 0.01f && std::fabs(g[18] + 2.3525f) < 0.01f,
              "gravity model: left shoulder pitch -5.54, left elbow -2.35 N·m at the captured pose");
        float tilted[NBODY] = {};
        arm_gravity(captured, 0.004f, 0.3479f, tilted);
        check(std::fabs(tilted[15] - g[15]) > 0.5f, "the robot's tilt changes what holding costs");

        // Lifting the left arm forward, elbow bent: gravity alone is ~7 N·m here, and the arm
        // trails its command by 0.15 rad. Before 2026-10-16 this tripped the guard.
        Controller c{Config{}};
        Inputs in = standing();
        in.q[15] = -1.3f;
        in.q[18] = 1.2f;
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.handle("target left_shoulder_pitch -1.6", t, in);
        run(c, t, 1.5f, in, ev);                      // commanded -1.6, measured still -1.3
        float hold[NBODY] = {};
        arm_gravity(in.q, 0.0f, 0.0f, hold);
        check(std::fabs(hold[15]) > 0.3f * 25.0f * 0.85f,
              "the pose really costs close to 30% of the shoulder's 25 N·m just to hold");
        in.tau[15] = hold[15] * 1.2f;                 // the real left arm is ~20% heavier
        run(c, t, 0.5f, in, ev);
        check(!has(ev, "stall"), "holding the arm's own weight is not pushing into something");
        in.tau[15] = hold[15] - 9.0f;                 // and now something stops it
        run(c, t, 0.3f, in, ev);
        check(has(ev, "stall left_shoulder_pitch"), "the same pose with a real push still stalls");
    }
    // Rule 9: a hot motor releases.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 0.5f, in, ev);
        in.temp[L_ELBOW] = 76;
        run(c, t, 0.1f, in, ev);
        check(has(ev, "releasing hot_left_elbow") && c.state() == State::Down,
              "a motor at 76 °C releases control");
    }
    // Rule 10: fingers.
    {
        Controller c{Config{}};
        Inputs in = standing();
        in.hand_ok[0] = true;                         // left answers, right does not
        for (int f = 0; f < 6; ++f) in.hand_pos[0][f] = f == 0 ? 399 : 50;
        long long t = 0;
        check(starts(c.handle("hand left 2 500", t, in), "err not in control"),
              "a finger is refused before take");
        c.handle("take", t, in);
        int pos[6], spd[6];
        check(!c.hand_due(0, t, pos, spd), "taking control writes nothing to the hand");
        Controller still{Config{}};
        still.handle("take", t, in);
        still.handle("release", t + 10, in);
        check(!still.hand_due(0, t + 100, pos, spd), "a hand never moved is not written on release");
        check(c.hand_goal(0, 0) == 399 && c.hand_goal(0, 2) == 50,
              "finger targets start where the fingers are");
        check(starts(c.handle("hand right 2 500", t, in), "err right hand not available"),
              "a hand that did not answer at take is refused");
        check(starts(c.handle("hand left 6 500", t, in), "err no such finger"), "finger 6 refused");
        check(starts(c.handle("hand left 2 nan", t, in), "err hand needs"), "NaN refused");
        check(starts(c.handle("hand left 2 1500", t, in), "ok hand left 2 1000"),
              "position clamped to 1000");
        check(c.hand_due(0, t, pos, spd) && pos[2] == 1000 && pos[0] == 399 && spd[2] == 300,
              "due: all six targets, the others unchanged, at the slow default speed");
        c.hand_written(0, t, pos);
        check(!c.hand_due(0, t + 100, pos, spd), "nothing more once written");
        c.handle("hand left 3 -20", t + 10, in);
        check(!c.hand_due(0, t + 20, pos, spd), "at most one write per 50 ms");
        check(c.hand_due(0, t + 60, pos, spd) && pos[3] == 0, "then the newest, clamped to 0");
        c.handle("release", t + 70, in);
        check(c.hand_due(0, t + 200, pos, spd) && pos[0] == 399 && pos[2] == 50 && pos[3] == 50,
              "release sends the hand back to where it was when control was taken");
        c.hand_written(0, t + 200, pos);
        check(!c.hand_due(0, t + 300, pos, spd), "once, then nothing more");
        check(starts(c.handle("hand left 2 500", t + 300, in), "err not in control"),
              "no finger target is taken while releasing");
    }
    std::printf(failures ? "FAILED: %d\n" : "PASS\n", failures);
    return failures ? 1 : 0;
}
