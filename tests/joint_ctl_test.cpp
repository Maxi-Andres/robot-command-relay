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
        check(starts(c.handle("target left_shoulder_pitch 0.5", t, in), "err joint not allowed"),
              "joint outside the allowlist refused");
        check(starts(c.handle("target waist_pitch 0.1", t, in), "err joint not allowed"),
              "locked waist joint refused");
        check(starts(c.handle("target left_elbow nan", t, in), "err target needs"), "NaN refused");
        check(starts(c.handle("target left_elbow 9", t, in), "ok target left_elbow 1.5000"),
              "target clamped to 0.6 rad from where it was taken");
        Config wide;
        wide.max_excursion = 5.0f;
        Controller d{wide};
        d.handle("take", t, in);
        check(starts(d.handle("target left_elbow 9", t, in), "ok target left_elbow 2.0444"),
              "target clamped to the URDF range minus the margin");
        check(starts(d.handle("target left_elbow -9", t, in), "ok target left_elbow -0.9972"),
              "and on the low side");
    }
    // Rule 5: speed.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.handle("target left_elbow 1.3", t, in);
        Output o = run(c, t, 0.4f, in, ev);
        const float moved = o.q[L_ELBOW_ARM] - 0.9f;
        check(moved > 0.19f && moved < 0.21f, "0.5 rad/s: 0.2 rad in 0.4 s");
        o = run(c, t, 1.0f, in, ev);
        check(o.q[L_ELBOW_ARM] > 1.299f && o.q[L_ELBOW_ARM] < 1.301f, "and stops at the target");
        check(o.q[L_ELBOW_ARM + 7] == in.q[L_ELBOW + 7], "the other elbow did not move");
    }
    // Rule 6: dead-man — freeze, then release.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        run(c, t, 2.1f, in, ev);
        c.handle("target left_elbow 1.4", t, in);
        Output o = run(c, t, 1.2f, in, ev, false);    // no keepalive from here on
        check(has(ev, "deadman_freeze"), "frozen after 1 s of silence");
        const float frozen = o.q[L_ELBOW_ARM];
        check(frozen < 1.4f, "frozen short of the target");
        o = run(c, t, 1.0f, in, ev, false);
        check(o.q[L_ELBOW_ARM] == frozen, "and stays there");
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
    std::printf(failures ? "FAILED: %d\n" : "PASS\n", failures);
    return failures ? 1 : 0;
}
