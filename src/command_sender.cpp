// command_sender — the ONLY process allowed to move the robot from a remote command.
//
// Reads one command per line on stdin and dispatches it through the Unitree SDK's
// SportClient. Runs ON THE ROBOT, next to its DDS; the HTTP layer that feeds it
// (relay_server.py) never touches DDS itself.
//
// Safety is built in HERE, not in the HTTP layer, because this process is the last thing
// between the network and the robot's legs:
//
//   1. ALLOWLIST BY CONSTRUCTION. There is no generic api_id passthrough. Only the verbs
//      in the dispatch table below exist, and acrobatics (flips, jumps, handstand, dances)
//      are deliberately absent — they can hurt the robot or a bystander.
//   2. VELOCITY CLAMP. move is clamped to MAX_VX / MAX_VY / MAX_VYAW whatever the caller
//      asks for.
//   3. DEAD-MAN SWITCH. A movement must be refreshed within DEADMAN_MS or StopMove is sent
//      automatically. A dropped link or a hung caller stops the robot instead of leaving it
//      running.
//   4. EOF STOPS THE ROBOT. If the HTTP layer dies, stdin closes, and StopMove is sent
//      before exiting rather than leaving the last command latched.
//
// Protocol (whitespace-separated, one command per line) — deliberately NOT JSON: the only
// producer is relay_server.py, and a hand-written JSON parser here would be pure attack
// surface for no benefit.
//
//   move <vx> <vy> <vyaw>
//   stop_move | stand_up | stand_down | damp | balance_stand | recovery_stand
//   sit | rise_sit | hello | keepalive
//
// Answers one line per command on stdout: "ok <verb>" or "err <reason>".

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace unitree::robot;

static float env_f(const char* k, float d) {
    const char* v = getenv(k);
    return (v && *v) ? strtof(v, nullptr) : d;
}

// A POINTER, not a global object: SportClient's constructor touches SDK internals that do
// not exist until ChannelFactory::Init() has run, and a global would be constructed before
// main() — which segfaults during static initialisation, before a single line is logged.
static std::unique_ptr<go2::SportClient> g_sport;
static std::mutex g_mu;                       // SportClient is not thread-safe
static std::atomic<long long> g_last_move_ms{0};
static std::atomic<bool> g_moving{false};
static std::atomic<bool> g_running{true};

static float MAX_VX, MAX_VY, MAX_VYAW;
static long DEADMAN_MS;

static long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static float clamp(float v, float lim) {
    if (std::isnan(v)) return 0.0f;
    return v > lim ? lim : (v < -lim ? -lim : v);
}

// Every verb the relay can perform. Anything absent here cannot be commanded at all.
static const std::map<std::string, std::function<int32_t()>> VERBS = {
    {"stop_move",      [] { return g_sport->StopMove(); }},
    {"stand_up",       [] { return g_sport->StandUp(); }},
    {"stand_down",     [] { return g_sport->StandDown(); }},
    {"damp",           [] { return g_sport->Damp(); }},
    {"balance_stand",  [] { return g_sport->BalanceStand(); }},
    {"recovery_stand", [] { return g_sport->RecoveryStand(); }},
    {"sit",            [] { return g_sport->Sit(); }},
    {"rise_sit",       [] { return g_sport->RiseSit(); }},
    {"hello",          [] { return g_sport->Hello(); }},
};

static void deadman_loop() {
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!g_moving.load()) continue;
        if (now_ms() - g_last_move_ms.load() < DEADMAN_MS) continue;
        {
            std::lock_guard<std::mutex> lk(g_mu);
            g_sport->StopMove();
        }
        g_moving.store(false);
        std::cout << "ev deadman_stop" << std::endl;
    }
}

int main() {
    const char* nic = getenv("DDS_IFACE");
    const std::string iface = (nic && *nic) ? nic : "eth0";
    MAX_VX = env_f("MAX_VX", 0.6f);
    MAX_VY = env_f("MAX_VY", 0.4f);
    MAX_VYAW = env_f("MAX_VYAW", 1.0f);
    DEADMAN_MS = (long)env_f("DEADMAN_MS", 1500);

    std::cerr << "[sender] iface=" << iface << " clamp vx=" << MAX_VX
              << " vy=" << MAX_VY << " vyaw=" << MAX_VYAW
              << " deadman=" << DEADMAN_MS << "ms" << std::endl;

    // Binding the interface is mandatory: Init(0, iface) alone receives nothing.
    // Wrapped because the most likely misconfiguration by far is a wrong DDS_IFACE, and the
    // SDK answers that with an uncaught DdsException — a core dump instead of a message.
    try {
        ChannelFactory::Instance()->Init(0, iface);
        // Constructed only now — AFTER the channel factory exists. See the note on g_sport.
        g_sport = std::make_unique<go2::SportClient>();
        g_sport->SetTimeout(5.0f);
        g_sport->Init();
    } catch (const std::exception& e) {
        std::cerr << "[sender] cannot init DDS on interface '" << iface << "': " << e.what()
                  << "\n[sender] set DDS_IFACE to an interface that exists on this machine "
                     "(the robot's internal bus is normally eth0)" << std::endl;
        return 2;
    }

    std::thread deadman(deadman_loop);

    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream is(line);
        std::string verb;
        if (!(is >> verb) || verb.empty()) continue;

        if (verb == "keepalive") {
            if (g_moving.load()) g_last_move_ms.store(now_ms());
            std::cout << "ok keepalive" << std::endl;
            continue;
        }

        if (verb == "move") {
            float vx = 0, vy = 0, vyaw = 0;
            if (!(is >> vx >> vy >> vyaw)) {
                std::cout << "err move needs vx vy vyaw" << std::endl;
                continue;
            }
            vx = clamp(vx, MAX_VX);
            vy = clamp(vy, MAX_VY);
            vyaw = clamp(vyaw, MAX_VYAW);
            int32_t r;
            {
                std::lock_guard<std::mutex> lk(g_mu);
                r = g_sport->Move(vx, vy, vyaw);
            }
            const bool zero = (vx == 0 && vy == 0 && vyaw == 0);
            g_moving.store(!zero);
            g_last_move_ms.store(now_ms());
            // Echo the CLAMPED values: the audit log upstream records what was asked for,
            // and a log that says 99 m/s when the robot got 0.6 is worse than no log.
            std::cout << (r == 0 ? "ok move " : "err move ") << r
                      << " applied=" << vx << "," << vy << "," << vyaw << std::endl;
            continue;
        }

        auto it = VERBS.find(verb);
        if (it == VERBS.end()) {
            std::cout << "err unknown verb" << std::endl;   // never reaches the robot
            continue;
        }
        int32_t r;
        {
            std::lock_guard<std::mutex> lk(g_mu);
            r = it->second();
        }
        if (verb == "stop_move" || verb == "damp") g_moving.store(false);
        std::cout << (r == 0 ? "ok " : "err ") << verb << " " << r << std::endl;
    }

    // stdin closed: the HTTP layer is gone. Never leave a movement latched.
    std::cerr << "[sender] stdin closed — stopping the robot" << std::endl;
    {
        std::lock_guard<std::mutex> lk(g_mu);
        g_sport->StopMove();
    }
    g_running.store(false);
    deadman.join();
    return 0;
}
