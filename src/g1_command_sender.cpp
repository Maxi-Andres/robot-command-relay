// g1_command_sender — the G1's command sender: its SDK client and its verbs, nothing else.
//
// The safety envelope (allowlist by construction, velocity clamp, dead-man switch, EOF stops
// the robot) is the SAME code the Go2 runs — sender_core.hpp; read that file first. Runs on
// the G1's PC2 (the Jetson): PC1 has no SSH, and PC2 shares the internal bus with it.
//
// THE G1 FALLS. A quadruped that loses power lies down; a humanoid that loses power falls over.
// So this table is narrower than the Go2's, and it is NOT the SDK's convenience calls: on THIS
// robot two of those are wrong. The ids below are the ones MEASURED on it and recorded in
// unitree_ros2/robot_executor/g1_commands.py (FSM_IDS) — the verb names are that file's skill
// names, so the executor maps a skill to a verb one to one. Change one, change both.
//
//   KEPT      stop_move      velocity 0. The stop.
//             stand_up       FSM 4   — observed: the Unitree app's Ready/Preparation.
//             walk_waist     FSM 501 — observed: walk on THIS robot (3-DoF waist). `move`
//                            does nothing until the robot is in a locomotion mode.
//             squat          FSM 706 — confirmed off the wire: the app's Squat AND Squat up,
//                            one toggle (down and parked, or up and back to locomotion).
//             lie_up         FSM 702 — confirmed: the app's Lie up, getting up off the floor.
//             balance_stand  balance mode 0 (stand in place, no stepping).
//             high_stand / low_stand   stand height, max / min.
//             wave_hand      the built-in gesture, one shot (needs a locomotion mode).
//
//   EXCLUDED  squat via the SDK's Squat()  FSM 2 — observed HALF-FALLING on this robot.
//             start          FSM 500 — the SDK's Start(), the 1-DoF-waist variant's walk; this
//                            robot's is 501. Not verified here, so not sent.
//             damp           FSM 1 — limp: collapses from any standing posture. The executor
//                            marks it dangerous too; it is not a remote stop for a humanoid.
//             zero_torque    FSM 0 — every motor limp at once.
//             sit            FSM 3 — the SDK's, never observed on this robot.
//             run, climb, set_fsm_id, set_speed_mode, switch_mode, user ctrl, shake_hand
//                            (a two-stage gesture a dropped link would leave half-done).
//
// `move` uses the SDK's non-continuous mode: the robot itself drops the velocity after 1 s.
// That is a second dead-man, on the robot, under the one in sender_core.hpp.
//
// CLAMP DEFAULTS ARE LOWER THAN THE GO2'S (0.3 / 0.2 / 0.5 against 0.6 / 0.4 / 1.0): first
// drives of a humanoid by a remote operator. Raise them in relay.env once it has walked.

#include "sender_core.hpp"

#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <memory>

using namespace unitree::robot;

int main() {
    // Built inside make(), i.e. after ChannelFactory::Init — see sender::Robot.
    std::shared_ptr<g1::LocoClient> c;
    return sender::run("g1-sender", {0.3f, 0.2f, 0.5f}, [&c] {
        c = std::make_shared<g1::LocoClient>();
        c->SetTimeout(5.0f);
        c->Init();
        // Every verb the relay can perform. Anything absent here cannot be commanded at all.
        const std::map<std::string, sender::Call> VERBS = {
            {"stop_move",      [c] { return c->StopMove(); }},
            {"stand_up",       [c] { return c->SetFsmId(4); }},
            {"walk_waist",     [c] { return c->SetFsmId(501); }},
            {"squat",          [c] { return c->SetFsmId(706); }},
            {"lie_up",         [c] { return c->SetFsmId(702); }},
            {"balance_stand",  [c] { return c->BalanceStand(); }},
            {"high_stand",     [c] { return c->HighStand(); }},
            {"low_stand",      [c] { return c->LowStand(); }},
            {"wave_hand",      [c] { return c->WaveHand(); }},
        };
        sender::Robot r;
        r.move = [c](float vx, float vy, float vyaw) { return c->Move(vx, vy, vyaw, false); };
        r.stop_move = [c] { return c->StopMove(); };
        r.verbs = VERBS;
        // A posture change ends locomotion: after these, no movement is in flight.
        r.stops_motion = {"stop_move", "squat", "lie_up"};
        return r;
    });
}
