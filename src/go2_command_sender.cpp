// go2_command_sender — the Go2's command sender: its SDK client and its verbs, nothing else.
//
// The safety envelope (allowlist by construction, velocity clamp, dead-man switch, EOF stops
// the robot) is shared with the G1 and lives in sender_core.hpp; read that file first.
// This was command_sender.cpp until 2026-10-01, when the G1 got its own.
//
// Acrobatics (flips, jumps, handstand, dances) are deliberately absent from the table: they
// can hurt the robot or a bystander, and the relay must not be how one gets triggered.

#include "sender_core.hpp"

#include <unitree/robot/go2/sport/sport_client.hpp>

#include <memory>

using namespace unitree::robot;

int main() {
    // Built inside make(), i.e. after ChannelFactory::Init — see sender::Robot.
    std::shared_ptr<go2::SportClient> c;
    return sender::run("go2-sender", {0.6f, 0.4f, 1.0f}, [&c] {
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
        };
        sender::Robot r;
        r.move = [c](float vx, float vy, float vyaw) { return c->Move(vx, vy, vyaw); };
        r.stop_move = [c] { return c->StopMove(); };
        r.verbs = VERBS;
        r.stops_motion = {"stop_move", "damp"};
        return r;
    });
}
