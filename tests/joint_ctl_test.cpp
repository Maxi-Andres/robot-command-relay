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
    // A trajectory point: several joints at once, same rules, all or nothing.
    {
        Controller c{Config{}};
        Inputs in = standing();
        long long t = 0;
        check(starts(c.handle("targets left_elbow 1.0", t, in), "err not in control"),
              "targets refused before take");
        c.handle("take", t, in);
        check(starts(c.handle("targets left_elbow 1.0 left_shoulder_pitch 0.3", t, in),
                     "ok targets 2"), "two joints in one point");
        check(c.goal(index_of("left_elbow")) == 1.0f &&
                  c.goal(index_of("left_shoulder_pitch")) == 0.3f,
              "both goals set");
        check(starts(c.handle("targets left_elbow 1.5 waist_pitch 0.1", t, in),
                     "err joint not allowed: waist_pitch"), "a locked joint refuses the point");
        check(c.goal(index_of("left_elbow")) == 1.0f, "and the point is not half applied");
        check(starts(c.handle("targets left_elbow nan", t, in), "err targets needs"),
              "NaN refused");
        check(starts(c.handle("targets left_elbow", t, in), "err targets needs"),
              "a joint without an angle refused");
        check(starts(c.handle("targets", t, in), "err targets needs"), "an empty point refused");
        c.handle("targets right_elbow 9", t, in);
        check(std::fabs(c.goal(index_of("right_elbow")) - 2.0444f) < 1e-3f,
              "clamped to the URDF range minus the margin, like target");
    }
    // Rule 12: grasp by touch.
    {
        auto hand_in = []() {
            Inputs in = standing();
            in.hand_ok[1] = true;                                   // the right hand answers
            for (int f = 0; f < 6; ++f) in.hand_pos[1][f] = f < 2 ? 400 : 50;
            in.touch_ok[1] = true;
            for (int k = 0; k < 5; ++k) in.touch_valid[1][k] = true;
            return in;
        };
        int pos[6], spd[6];
        // Held: the thumb and the index touch. Every flexing finger stops where it is + squeeze.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            check(starts(c.handle("grasp right", t, in), "err not in control"),
                  "grasp refused before take");
            c.handle("take", t, in);
            check(starts(c.handle("grasp left", t, in), "err left hand not available"),
                  "a hand that did not answer at take cannot grasp");
            check(starts(c.handle("grasp right -1", t, in), "err grasp force"), "a bad force refused");
            check(starts(c.handle("grasp right", t, in), "ok grasp right 1.0000"), "grasp accepted");
            check(c.hand_due(1, t, pos, spd) && pos[0] == 1000 && pos[2] == 1000 && pos[5] == 1000 &&
                      pos[1] == 400,
                  "closes the flexing fingers, not the thumb's rotation");
            c.hand_written(1, t, pos);
            std::vector<std::string> ev;
            run(c, t, 0.2f, in, ev);
            check(!has(ev, "grasp"), "nothing decided while nothing touches");
            for (int f = 0; f < 6; ++f) in.hand_pos[1][f] = 600;
            in.touch_n[1][0] = 1.5f;                                 // thumb
            run(c, t, 0.1f, in, ev);
            check(!has(ev, "grasp"), "the thumb alone is not a grasp");
            in.touch_n[1][1] = 2.0f;                                 // index
            run(c, t, 0.1f, in, ev);
            check(has(ev, "grasp right held 1.50 2.00 0.00 0.00 0.00"), "thumb + index = held");
            int ma[6];
            check(!c.hand_due(1, t + 1000, pos, spd), "held: no position write, it would end the squeeze");
            check(c.grip_due(1, t + 1000, ma) && ma[0] == 250 && ma[2] == 250 && ma[3] == 0 &&
                      ma[1] == 0,
                  "the touching fingers squeeze at grasp_current_ma, the rest and the rotation do not");
            c.grip_written(1, t + 1000);
            check(!c.grip_due(1, t + 2000, ma), "the squeeze is written once");
            check(starts(c.handle("open right", t, in), "ok open right"), "open accepted");
            check(!c.gripping(1), "open ends the squeeze");
            check(c.hand_due(1, t + 2000, pos, spd) && pos[0] == 0 && pos[3] == 0 && pos[1] == 400,
                  "open: the flexing fingers to 0");
        }
        // A void reading never counts, however high.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right 0.5", t, in);
            in.touch_n[1][0] = 3.0f;
            in.touch_n[1][2] = 5.0f;
            in.touch_valid[1][2] = false;                            // the middle's stuck bit
            std::vector<std::string> ev;
            run(c, t, 0.2f, in, ev);
            check(!has(ev, "grasp"), "a void fingertip reading is not a touch");
            in.hand_state[1][3] = 2;                                 // ... but its motor stalls
            run(c, t, 0.1f, in, ev);
            check(has(ev, "grasp right held 3.00 0.00 -"), "a stalled motor counts, as the fallback");
        }
        // Empty, as the robot did it (2026-10-30): the thumb closes, the index and middle stop
        // against it at 605 and 671, everything idle, no touch.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right", t, in);
            for (int f : {0, 2, 3, 4, 5}) in.hand_state[1][f] = 1;      // running
            std::vector<std::string> ev;
            run(c, t, 0.4f, in, ev);
            check(!has(ev, "grasp"), "nothing decided while the fingers run");
            const int empty[6] = {999, 400, 605, 671, 999, 999};
            for (int f = 0; f < 6; ++f) { in.hand_pos[1][f] = empty[f]; in.hand_state[1][f] = 0; }
            run(c, t, 0.1f, in, ev);
            check(!has(ev, "grasp"), "stopped, but not yet for the settle time");
            run(c, t, 0.2f, in, ev);
            check(has(ev, "grasp right empty"), "all stopped and the thumb closed = empty");
        }
        // Held, as the robot did it on a bottle (2026-10-30): every finger stops short (68-277),
        // idle, current ~0, fingertips under 1 N — blocked is the contact.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right", t, in);
            for (int f : {0, 2, 3, 4, 5}) in.hand_state[1][f] = 1;
            std::vector<std::string> ev;
            run(c, t, 0.4f, in, ev);
            const int bottle[6] = {105, 400, 68, 262, 277, 91};
            for (int f = 0; f < 6; ++f) { in.hand_pos[1][f] = bottle[f]; in.hand_state[1][f] = 0; }
            in.touch_n[1][3] = 0.28f;
            run(c, t, 0.3f, in, ev);
            check(has(ev, "grasp right held 0.00 0.00 0.00 0.28 0.00"),
                  "fingers stopped short of closing = blocked by the object = held");
            int ma[6];
            check(c.grip_due(1, t + 1000, ma) && ma[0] == 250 && ma[2] == 250 && ma[5] == 250,
                  "and squeezes every finger that stopped on it");
            c.handle("release", t, in);
            check(!c.gripping(1) && c.hand_due(1, t + 2000, pos, spd) && pos[2] == 50,
                  "the release ends the squeeze: a position write back to where it started");
        }
        // Thumb first: the rotation turns, then the flexing fingers close.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            check(starts(c.handle("grasp right 1 2000", t, in), "err grasp thumb"),
                  "a thumb rotation outside 0-1000 refused");
            check(starts(c.handle("grasp right 1 900", t, in), "ok grasp right"), "with a thumb");
            check(c.hand_due(1, t, pos, spd) && pos[1] == 900 && pos[0] == 400 && pos[2] == 50,
                  "first only the thumb's rotation moves");
            c.hand_written(1, t, pos);
            std::vector<std::string> ev;
            run(c, t, 0.2f, in, ev);
            check(!c.hand_due(1, t, pos, spd), "nothing closes while the thumb turns");
            in.hand_pos[1][1] = 880;                               // close enough to 900
            run(c, t, 0.1f, in, ev);
            check(c.hand_due(1, t + 100, pos, spd) && pos[0] == 1000 && pos[2] == 1000 &&
                      pos[1] == 900,
                  "then the flexing fingers close, the thumb staying opposed");
        }
        // The fingers never move at all (the targets were not taken): no "held" out of nothing.
        {
            Config cfg;
            cfg.grasp_ms = 1000;
            Controller c{cfg};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right", t, in);
            std::vector<std::string> ev;
            run(c, t, 0.6f, in, ev);
            check(!has(ev, "grasp"), "a hand that never moved is not blocked: nothing decided");
            run(c, t, 0.5f, in, ev);
            check(has(ev, "grasp right uncertain") && !has(ev, "grasp right held"),
                  "it runs out the clock to uncertain, never held");
        }
        // Undecided: the time runs out; the fingers stop where they are.
        {
            Config cfg;
            cfg.grasp_ms = 500;
            Controller c{cfg};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right", t, in);
            c.hand_due(1, t, pos, spd);
            c.hand_written(1, t, pos);
            for (int f = 0; f < 6; ++f) in.hand_pos[1][f] = 500;
            std::vector<std::string> ev;
            run(c, t, 0.6f, in, ev);
            check(has(ev, "grasp right uncertain"), "undecided in grasp_ms = uncertain");
            check(c.hand_due(1, t + 1000, pos, spd) && pos[0] == 500 && pos[5] == 500,
                  "and it stops closing where it is");
        }
        // The thumb as a post (the operator's bottle pose): it turns and holds its flex; the four
        // fingers close; two of them blocked = held, and only they squeeze.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            check(starts(c.handle("grasp right 1 950 30", t, in), "ok grasp right"), "with a post");
            check(c.hand_due(1, t, pos, spd) && pos[1] == 950 && pos[0] == 30 && pos[2] == 50,
                  "first the thumb: rotation 950, flex 30");
            c.hand_written(1, t, pos);
            in.hand_pos[1][1] = 940;
            in.hand_pos[1][0] = 30;
            std::vector<std::string> ev;
            run(c, t, 0.1f, in, ev);
            check(c.hand_due(1, t + 100, pos, spd) && pos[0] == 30 && pos[2] == 1000 && pos[5] == 1000,
                  "then the four fingers close and the thumb's flex stays at 30");
            c.hand_written(1, t + 100, pos);
            for (int f : {2, 3, 4, 5}) in.hand_state[1][f] = 1;
            run(c, t, 0.3f, in, ev);
            const int bottle[6] = {30, 940, 400, 450, 500, 980};    // pinky got past it
            for (int f = 0; f < 6; ++f) { in.hand_pos[1][f] = bottle[f]; in.hand_state[1][f] = 0; }
            run(c, t, 0.3f, in, ev);
            check(has(ev, "grasp right held"), "three fingers stopped on it, the post still = held");
            int ma[6];
            check(c.grip_due(1, t + 1000, ma) && ma[0] == 0 && ma[1] == 0 && ma[2] == 250 &&
                      ma[4] == 250 && ma[5] == 0,
                  "the blocked fingers squeeze; the post, its rotation and the free pinky do not");
        }
        // With a post and an empty hand, the four fingers close all the way: empty.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right 1 950 30", t, in);
            in.hand_pos[1][1] = 950;
            std::vector<std::string> ev;
            run(c, t, 0.1f, in, ev);
            for (int f : {2, 3, 4, 5}) in.hand_state[1][f] = 1;
            run(c, t, 0.3f, in, ev);
            const int closed[6] = {30, 950, 990, 990, 990, 990};
            for (int f = 0; f < 6; ++f) { in.hand_pos[1][f] = closed[f]; in.hand_state[1][f] = 0; }
            run(c, t, 0.3f, in, ev);
            check(has(ev, "grasp right empty") && !has(ev, "grasp right held"),
                  "with a post: the four closed all the way = empty");
        }
        // A release ends a grasp: nothing is decided after it.
        {
            Controller c{Config{}};
            Inputs in = hand_in();
            long long t = 0;
            c.handle("take", t, in);
            c.handle("grasp right", t, in);
            c.handle("release", t, in);
            in.touch_n[1][0] = in.touch_n[1][1] = 3.0f;
            std::vector<std::string> ev;
            run(c, t, 0.1f, in, ev, false);
            check(!has(ev, "grasp"), "no grasp decided after a release");
        }
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
    // Rule 11: the arm's own weight goes out as torque, so the kp spring does not sag under it.
    {
        Inputs in = standing();
        in.q[15] = -1.3f;                             // left arm raised forward, elbow bent:
        in.q[18] = 1.2f;                              // ~7 N·m on the shoulder just to hold it
        float hold[NBODY] = {};
        arm_gravity(in.q, 0.0f, 0.0f, hold);

        Controller c{Config{}};
        long long t = 0;
        c.handle("take", t, in);
        std::vector<std::string> ev;
        Output o = run(c, t, 1.0f, in, ev);
        check(std::fabs(o.tau[15 - FIRST] - hold[15] * o.weight) < 1e-3f,
              "halfway up the hand-over, half the holding torque");
        o = run(c, t, 1.1f, in, ev);
        check(std::fabs(o.tau[15 - FIRST] - hold[15]) < 1e-3f &&
                  std::fabs(o.tau[18 - FIRST] - hold[18]) < 1e-3f,
              "in control: the shoulder and elbow carry what holding the arm costs");
        check(o.tau[15 - FIRST] * hold[15] > 0.0f && std::fabs(hold[15]) > 5.0f,
              "same sign as the measured holding torque, and a real amount");
        check(o.tau[0] == 0.0f && o.tau[1] == 0.0f && o.tau[2] == 0.0f,
              "nothing on the waist: it holds no arm weight, two of it have no motor");

        Inputs sag = in;
        sag.q[15] = -1.15f;                           // the measured arm sags 0.15 rad
        o = run(c, t, 0.2f, sag, ev);
        check(std::fabs(o.tau[15 - FIRST] - hold[15]) < 1e-3f,
              "computed at the commanded pose, not the measured one: no feedback from the sag");

        Inputs bad = in;
        bad.rpy[1] = std::nanf("");
        o = run(c, t, 0.1f, bad, ev);
        check(o.tau[15 - FIRST] == 0.0f && c.state() == State::Active,
              "no tilt reading: no feedforward for that tick, control kept");

        c.handle("release", t, in);
        o = run(c, t, 1.0f, in, ev);
        check(std::fabs(o.tau[15 - FIRST] - hold[15] * o.weight) < 1e-3f && o.weight < 0.55f,
              "the release ramps it down with the weight");
        for (int k = 0; k < 100 && !c.finished(); ++k) {
            t += 20;
            o = c.tick(t, 0.02f, in, ev);
        }
        bool zero = true;
        for (float v : o.tau) zero = zero && v == 0.0f;
        check(o.weight == 0.0f && zero, "the last message carries no torque at all");

        Config off;
        off.ff_gain = 0.0f;
        Controller d{off};
        long long u = 0;
        d.handle("take", u, in);
        o = run(d, u, 2.1f, in, ev);
        zero = true;
        for (float v : o.tau) zero = zero && v == 0.0f;
        check(zero, "JOINT_CTL_FEEDFORWARD=0: no torque, as before");

        Config capped;
        capped.ff_max_frac = 0.1f;                    // 2.5 N·m on a 25 N·m joint
        Controller e{capped};
        long long w = 0;
        e.handle("take", w, in);
        o = run(e, w, 2.1f, in, ev);
        check(std::fabs(o.tau[15 - FIRST]) <= 2.5f + 1e-4f && std::fabs(o.tau[15 - FIRST]) > 2.4f,
              "never more than ff_max_frac of the rated effort");
    }
    std::printf(failures ? "FAILED: %d\n" : "PASS\n", failures);
    return failures ? 1 : 0;
}
