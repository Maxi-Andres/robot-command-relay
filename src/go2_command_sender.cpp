// go2_command_sender — the Go2's command sender: its SDK client and its verbs, nothing else.
//
// The safety envelope (allowlist by construction, velocity clamp, dead-man switch, EOF stops
// the robot) is shared with the G1 and lives in sender_core.hpp; read that file first.
// This was command_sender.cpp until 2026-10-01, when the G1 got its own.
//
// Every action the SDK offers is in the table since 2026-10-01 (the operator's call), the
// acrobatics included — flips, jumps, handstand, dances. They can hurt the robot or a
// bystander, so the gate is the executor's SAFE MODE (DANGEROUS_SKILLS): it refuses them until
// the operator turns Safe off. This process does not repeat that gate.

#include "sender_core.hpp"

#include <unitree/robot/go2/sport/sport_client.hpp>

#include <memory>

using namespace unitree::robot;

int main() {
    // Built inside make(), i.e. after ChannelFactory::Init — see sender::Robot.
    std::shared_ptr<go2::SportClient> c;
    return sender::run("go2-sender", {2.0f, 1.0f, 3.0f}, [&c] {
        c = std::make_shared<go2::SportClient>();
        c->SetTimeout(5.0f);
        c->Init();
        // Every verb the relay can perform. Anything absent here cannot be commanded at all.
        const std::map<std::string, sender::Call> VERBS = {
            {"stop_move",      [c] { return c->StopMove(); }},
            {"stand_up",       [c] { return c->StandUp(); }},
            {"stand_down",     [c] { return c->StandDown(); }},
            {"damp",           [c] { return c->Damp(); }},
            {"balance_stand",  [c] { return c->BalanceStand(); }},
            {"recovery_stand", [c] { return c->RecoveryStand(); }},
            {"sit",            [c] { return c->Sit(); }},
            {"rise_sit",       [c] { return c->RiseSit(); }},
            {"hello",          [c] { return c->Hello(); }},
            // Gestures with all four feet on the ground or sitting: no jump, no flip.
            {"stretch",        [c] { return c->Stretch(); }},
            {"scrape",         [c] { return c->Scrape(); }},
            {"heart",          [c] { return c->Heart(); }},
            // Pose is a mode, not a gesture: while it is on, the robot holds its feet and a
            // move tilts the body instead of walking. One verb per side because verbs carry
            // no arguments — the sender's whole input is "verb [vx vy vyaw]".
            {"pose_on",        [c] { return c->Pose(true); }},
            {"pose_off",       [c] { return c->Pose(false); }},
            // SAFE-GATED in the executor: whole-body routines and acrobatics.
            {"dance1",         [c] { return c->Dance1(); }},
            {"dance2",         [c] { return c->Dance2(); }},
            {"front_jump",     [c] { return c->FrontJump(); }},
            {"front_pounce",   [c] { return c->FrontPounce(); }},
            {"front_flip",     [c] { return c->FrontFlip(); }},
            {"back_flip",      [c] { return c->BackFlip(); }},
            {"left_flip",      [c] { return c->LeftFlip(); }},
            {"handstand_on",   [c] { return c->HandStand(true); }},
            {"handstand_off",  [c] { return c->HandStand(false); }},
            {"walk_upright_on",  [c] { return c->WalkUpright(true); }},
            {"walk_upright_off", [c] { return c->WalkUpright(false); }},
            // Gaits (go2_commands.GO2_GAIT_API): which controller the next `move` walks with.
            {"gait_classic",     [c] { return c->ClassicWalk(true); }},
            {"gait_free_walk",   [c] { return c->FreeWalk(); }},
            {"gait_trot_run",    [c] { return c->TrotRun(); }},
            {"gait_static_walk", [c] { return c->StaticWalk(); }},
            {"gait_economic",    [c] { return c->EconomicGait(); }},
            {"gait_cross_step",  [c] { return c->CrossStep(true); }},
        };
        sender::Robot r;
        r.move = [c](float vx, float vy, float vyaw) { return c->Move(vx, vy, vyaw); };
        r.stop_move = [c] { return c->StopMove(); };
        r.verbs = VERBS;
        r.stops_motion = {"stop_move", "damp", "dance1", "dance2", "front_jump", "front_pounce",
                          "front_flip", "back_flip", "left_flip"};
        return r;
    });
}
