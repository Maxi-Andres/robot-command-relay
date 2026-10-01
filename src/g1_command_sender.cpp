// g1_command_sender — the G1's command sender: its SDK clients and its verbs, nothing else.
//
// The safety envelope (allowlist by construction, velocity clamp, dead-man switch, EOF stops
// the robot) is the SAME code the Go2 runs — sender_core.hpp; read that file first. Runs on
// the G1's PC2 (the Jetson): PC1 has no SSH, and PC2 shares the internal bus with it.
//
// THE G1 FALLS. A quadruped that loses power lies down; a humanoid that loses power falls over.
// The table is NOT the SDK's convenience calls: on THIS robot two of those are wrong. The ids
// below are the ones MEASURED on it, most read off its bus on 2026-10-01 while the Unitree app
// drove it, and recorded in unitree_ros2/robot_executor/g1_commands.py (FSM_IDS).
//
// WHAT CAN DROP IT IS IN HERE ON PURPOSE (2026-10-01, the operator's call): damp, zero_torque
// and the dances are reachable, and the gate is the executor's SAFE MODE (DANGEROUS_SKILLS),
// which refuses them until the operator turns Safe off. This process does not repeat that gate.
//
//   stop_move      velocity 0, AND ends a running arm action (api 7100, below): mid-action
//                  the robot ignores velocity and mode changes alike (seen 2026-10-01).
//   stand_up       FSM 4   — the app's Ready/Preparation.
//   start          FSM 500 — Walk, waist LOCKED.  } ONE walk and ONE run exist at a time:
//   walk_waist     FSM 501 — Walk, waist free.    } the app's waist lock picks them and this
//   run            FSM 801 — Run, waist LOCKED.   } process cannot read it, so relay.env
//   run_waist      FSM 802 — Run, waist free.     } declares it (G1_WAIST_LOCK=1 = locked).
//                  Toggling the lock publishes nothing; it only changes which id the app sends.
//   climb          FSM 812 — the app's Climb (observed with the waist free).
//   squat          FSM 706 — Squat AND Squat up, one toggle. Down, the robot drops to FSM 1 by
//                  itself ~9 s later; up, it lands in Run (801), not Walk.
//   lie_up         FSM 702 — getting up off the floor.
//   damp / zero_torque  FSM 1 / 0 — limp. SAFE-GATED in the executor.
//   balance_stand, high_stand, low_stand, wave_hand — the SDK's own calls.
//   action_*       the app's arm actions: SetFsmId(550000 + code); the robot sits in FSM 550
//                  while it plays and returns to Run by itself. Codes read off the bus. Played
//                  only from Run: from Walk the robot answers 0 and does NOTHING, so the FSM
//                  is read first and 7404 returned instead. Pressed again while one plays, it
//                  ends it — the app's own toggle (shake_hand holds the hand out until then).
//   arm_*          the arm actions the app was not watched sending, by the ARM service's id
//                  (api 7106) — the ids the robot itself published (GetActionList).
//   dance_*        the robot's stored routines, by name (arm api 7108). SAFE-GATED.
//   stop_dance     arm api 7113.
//
// LEFT OUT: high five (550541 — the robot started falling BACKWARDS, 2026-10-01, and Safe gates
// whole skills, not one arm action); the SDK's Squat() (FSM 2, half-falls); sit (FSM 3, never
// observed here); the raw modes (set_fsm_id, set_speed_mode, switch_mode) — they take a
// number, and a verb carries none.
//
// `move` uses the SDK's non-continuous mode: the robot itself drops the velocity after 1 s.
// That is a second dead-man, on the robot, under the one in sender_core.hpp.
//
// CLAMPS (1.2 / 0.5 / 1.2, the Go2's 2.0 / 1.0 / 3.0) match the drive pad's fast preset; they
// were 0.3 / 0.2 / 0.5 for the first drives and made all three presets the same speed
// (2026-10-01). relay.env's MAX_* override them. The robot's controller caps each mode on top.

#include "sender_core.hpp"

#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <memory>

using namespace unitree::robot;

namespace {

// The run FSM of each waist setting: an action is only played from here.
constexpr int RUN_LOCKED = 801, RUN_FREE = 802;
// The FSM the robot reports while an app action plays.
constexpr int IN_ACTION = 550;
// The app's "end the running action" (loco api 7100, ROBOT_API_ID_LOCO_FSM_API), verbatim.
const char* const END_ACTION = R"({"fsm_id":550,"api_id":2,"motion_paused":false})";
// Unitree's own code for "arm actions only work in certain fsm ids" (g1_arm_action_error.hpp).
constexpr int32_t NOT_IN_RUN = 7404;

}  // namespace

int main() {
    // Built inside make(), i.e. after ChannelFactory::Init — see sender::Robot.
    std::shared_ptr<g1::LocoClient> c;
    std::shared_ptr<g1::G1ArmActionClient> arm;
    return sender::run("g1-sender", {1.2f, 0.5f, 1.2f}, [&c, &arm] {
        c = std::make_shared<g1::LocoClient>();
        c->SetTimeout(5.0f);
        c->Init();
        arm = std::make_shared<g1::G1ArmActionClient>();
        arm->SetTimeout(5.0f);
        arm->Init();

        const char* lock = getenv("G1_WAIST_LOCK");
        const bool locked = lock && std::string(lock) == "1";
        const int run_fsm = locked ? RUN_LOCKED : RUN_FREE;
        auto end_action = [c] {
            std::string data;
            return c->_fsm_api(END_ACTION, data);
        };
        // An app action: from Run it starts, while one plays it ends it, anywhere else 7404.
        auto action = [c, run_fsm, end_action](int code) -> sender::Call {
            return [c, run_fsm, end_action, code] {
                int fsm = 0;
                if (c->GetFsmId(fsm) != 0) return NOT_IN_RUN;
                if (fsm == IN_ACTION) return end_action();
                if (fsm != run_fsm) return NOT_IN_RUN;
                return c->SetFsmId(550000 + code);
            };
        };
        auto arm_id = [arm](int32_t id) -> sender::Call {
            return [arm, id] { return arm->ExecuteAction(id); };
        };
        auto dance = [arm](const char* name) -> sender::Call {
            return [arm, name] { return arm->ExecuteAction(std::string(name)); };
        };
        // Every verb the relay can perform. Anything absent here cannot be commanded at all.
        std::map<std::string, sender::Call> VERBS = {
            {"stop_move",      [c, end_action] {
                end_action();                    // a no-op unless an action is playing
                return c->StopMove();
            }},
            {"stand_up",       [c] { return c->SetFsmId(4); }},
            {"walk_waist",     [c] { return c->SetFsmId(501); }},
            {"start",          [c] { return c->SetFsmId(500); }},
            {"run",            [c] { return c->SetFsmId(RUN_LOCKED); }},
            {"run_waist",      [c] { return c->SetFsmId(RUN_FREE); }},
            {"climb",          [c] { return c->SetFsmId(812); }},
            {"squat",          [c] { return c->SetFsmId(706); }},
            {"lie_up",         [c] { return c->SetFsmId(702); }},
            {"damp",           [c] { return c->SetFsmId(1); }},
            {"zero_torque",    [c] { return c->SetFsmId(0); }},
            {"balance_stand",  [c] { return c->BalanceStand(); }},
            {"high_stand",     [c] { return c->HighStand(); }},
            {"low_stand",      [c] { return c->LowStand(); }},
            {"wave_hand",      [c] { return c->WaveHand(); }},
            {"action_hug",           action(542)},
            {"action_clap",          action(565)},
            {"action_face_wave",     action(535)},
            {"action_left_kiss",     action(527)},
            {"action_heart",         action(561)},
            {"action_hands_up",      action(529)},
            {"action_x_ray",         action(563)},
            {"action_right_hand_up", action(550)},
            {"action_reject",        action(547)},
            {"action_shake_hand",    action(552)},
            {"arm_release_arm",      arm_id(99)},
            {"arm_turn_back_wave",   arm_id(1)},
            {"arm_two_hand_kiss",    arm_id(11)},
            {"arm_right_kiss",       arm_id(13)},
            {"arm_right_heart",      arm_id(21)},
            {"arm_high_wave",        arm_id(26)},
            {"arm_box_win_left",     arm_id(28)},
            {"arm_box_win_right",    arm_id(29)},
            {"arm_box_win_both",     arm_id(30)},
            {"arm_hand_on_heart",    arm_id(33)},
            {"arm_hands_up_right",   arm_id(34)},
            {"arm_forward_push",     arm_id(36)},
            {"dance_waist_drum",     dance("Waist_Drum_Dance")},
            {"dance_scratch_head",   dance("Scratch_head")},
            {"dance_spin_discs",     dance("Spin_discs")},
            {"dance_throw_money",    dance("Throw_money")},
            {"stop_dance",     [arm] { return arm->StopCustomAction(); }},
        };
        VERBS.erase(locked ? "walk_waist" : "start");
        VERBS.erase(locked ? "run_waist" : "run");
        sender::Robot r;
        r.move = [c](float vx, float vy, float vyaw) { return c->Move(vx, vy, vyaw, false); };
        r.stop_move = [c] { return c->StopMove(); };
        r.verbs = VERBS;
        // These end locomotion: after them, no movement is in flight.
        r.stops_motion = {"stop_move", "squat", "lie_up", "damp", "zero_torque"};
        for (const auto& v : VERBS)
            if (v.first.rfind("action_", 0) == 0 || v.first.rfind("dance_", 0) == 0)
                r.stops_motion.insert(v.first);
        return r;
    });
}
