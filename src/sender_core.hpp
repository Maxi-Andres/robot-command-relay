// sender_core.hpp — the safety envelope every robot's command sender shares.
//
// A command sender is the ONLY process allowed to move a robot from a remote command. It reads
// one command per line on stdin and dispatches it through the Unitree SDK. It runs ON THE
// ROBOT, next to its DDS; the HTTP layer that feeds it (relay_server.py) never touches DDS.
//
// Everything that keeps the robot safe lives HERE, once, for every robot — what a robot file
// (go2_command_sender.cpp, g1_command_sender.cpp) adds is only its SDK client and its verbs:
//
//   1. ALLOWLIST BY CONSTRUCTION. There is no generic api_id passthrough. Only the verbs in a
//      robot's table exist; anything else is "err unknown verb" and never reaches the robot.
//   2. VELOCITY CLAMP. move is clamped to MAX_VX / MAX_VY / MAX_VYAW whatever is asked for.
//   3. DEAD-MAN SWITCH. A movement must be refreshed within DEADMAN_MS or StopMove is sent
//      automatically. A dropped link or a hung caller stops the robot instead of leaving it
//      running.
//   4. EOF STOPS THE ROBOT. If the HTTP layer dies, stdin closes, and StopMove is sent before
//      exiting rather than leaving the last command latched.
//   5. JOYSTICK ONLY INSIDE ITS MODE. A robot may offer `joy` (the Go2: its pose mode reads the
//      app's joystick topic, not the Move api). It is accepted only after one of the robot's
//      `joy_on` verbs and until ANY other verb — outside pose the same sticks would WALK the
//      robot, around every clamp above. The last value is re-published every 100 ms, like the
//      app does, and turned to zero once it is DEADMAN_MS old.
//
// Protocol (whitespace-separated, one command per line) — deliberately NOT JSON: the only
// producer is relay_server.py, and a hand-written JSON parser here would be pure attack surface.
//
//   move <vx> <vy> <vyaw>  |  joy <lx> <ly> <rx> <ry>  |  keepalive  |  <verb from the table>
//
// Answers one line per command on stdout: "ok <verb>" or "err <reason>".
#pragma once

#include <unitree/robot/channel/channel_factory.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>

namespace sender {

using Call = std::function<int32_t()>;

// What a robot file provides. Built by `make` AFTER ChannelFactory::Init: the SDK clients'
// constructors touch internals that do not exist before it, and a client built earlier (a
// global, say) segfaults during static initialisation, before a single line is logged.
struct Robot {
    std::function<int32_t(float, float, float)> move;
    Call stop_move;
    std::map<std::string, Call> verbs;    // the discrete verbs; move/keepalive are handled here
    std::set<std::string> stops_motion;   // verbs after which no movement is in flight
    // Optional: publish joystick sticks (each -1..1). Empty = the robot has no `joy`.
    std::function<void(float, float, float, float)> joy;
    std::set<std::string> joy_on;         // verbs after which `joy` is accepted
};

struct Limits {
    float vx, vy, vyaw;                   // per-robot defaults; MAX_VX/MAX_VY/MAX_VYAW override
};

inline float env_f(const char* k, float d) {
    const char* v = getenv(k);
    return (v && *v) ? strtof(v, nullptr) : d;
}

inline long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

inline float clamp(float v, float lim) {
    if (std::isnan(v)) return 0.0f;
    return v > lim ? lim : (v < -lim ? -lim : v);
}

// Runs the stdin protocol until EOF and returns the process exit code.
inline int run(const char* tag, Limits defaults, const std::function<Robot()>& make) {
    using namespace unitree::robot;
    const char* nic = getenv("DDS_IFACE");
    const std::string iface = (nic && *nic) ? nic : "eth0";
    const float max_vx = env_f("MAX_VX", defaults.vx);
    const float max_vy = env_f("MAX_VY", defaults.vy);
    const float max_vyaw = env_f("MAX_VYAW", defaults.vyaw);
    const long deadman_ms = (long)env_f("DEADMAN_MS", 1000);

    std::cerr << "[" << tag << "] iface=" << iface << " clamp vx=" << max_vx
              << " vy=" << max_vy << " vyaw=" << max_vyaw
              << " deadman=" << deadman_ms << "ms" << std::endl;

    // Binding the interface is mandatory: Init(0, iface) alone receives nothing. Wrapped
    // because the most likely misconfiguration by far is a wrong DDS_IFACE, and the SDK
    // answers that with an uncaught DdsException — a core dump instead of a message.
    Robot robot;
    try {
        ChannelFactory::Instance()->Init(0, iface);
        robot = make();
    } catch (const std::exception& e) {
        std::cerr << "[" << tag << "] cannot init DDS on interface '" << iface << "': "
                  << e.what() << "\n[" << tag << "] set DDS_IFACE to an interface that exists "
                     "on this machine (the robot's internal bus is normally eth0)" << std::endl;
        return 2;
    }

    std::mutex mu;                        // the SDK clients are not thread-safe
    std::atomic<long long> last_move_ms{0};
    std::atomic<bool> moving{false};
    std::atomic<bool> running{true};

    // Joystick state (rule 5). All of it is touched under `mu` only.
    bool joy_enabled = false, joy_live = false;
    long long last_joy_ms = 0;
    float joy_v[4] = {0, 0, 0, 0};
    auto joy_zero = [&] {                 // caller holds mu
        if (robot.joy && joy_live) robot.joy(0, 0, 0, 0);
        joy_live = false;
    };

    std::thread deadman([&] {
        while (running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            {
                std::lock_guard<std::mutex> lk(mu);
                if (joy_live) {
                    if (now_ms() - last_joy_ms < deadman_ms) {
                        robot.joy(joy_v[0], joy_v[1], joy_v[2], joy_v[3]);
                    } else {
                        joy_zero();
                        std::cout << "ev deadman_joy_zero" << std::endl;
                    }
                }
            }
            if (!moving.load()) continue;
            if (now_ms() - last_move_ms.load() < deadman_ms) continue;
            {
                std::lock_guard<std::mutex> lk(mu);
                robot.stop_move();
            }
            moving.store(false);
            std::cout << "ev deadman_stop" << std::endl;
        }
    });

    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream is(line);
        std::string verb;
        if (!(is >> verb) || verb.empty()) continue;

        if (verb == "keepalive") {
            if (moving.load()) last_move_ms.store(now_ms());
            std::cout << "ok keepalive" << std::endl;
            continue;
        }

        if (verb == "move") {
            float vx = 0, vy = 0, vyaw = 0;
            if (!(is >> vx >> vy >> vyaw)) {
                std::cout << "err move needs vx vy vyaw" << std::endl;
                continue;
            }
            vx = clamp(vx, max_vx);
            vy = clamp(vy, max_vy);
            vyaw = clamp(vyaw, max_vyaw);
            int32_t r;
            {
                std::lock_guard<std::mutex> lk(mu);
                r = robot.move(vx, vy, vyaw);
            }
            const bool zero = (vx == 0 && vy == 0 && vyaw == 0);
            moving.store(!zero);
            last_move_ms.store(now_ms());
            // Echo the CLAMPED values: the audit log upstream records what was asked for, and
            // a log that says 99 m/s when the robot got 0.6 is worse than no log.
            std::cout << (r == 0 ? "ok move " : "err move ") << r
                      << " applied=" << vx << "," << vy << "," << vyaw << std::endl;
            continue;
        }

        if (verb == "joy") {
            float v[4];
            if (!robot.joy) {
                std::cout << "err unknown verb" << std::endl;
                continue;
            }
            if (!(is >> v[0] >> v[1] >> v[2] >> v[3])) {
                std::cout << "err joy needs lx ly rx ry" << std::endl;
                continue;
            }
            std::lock_guard<std::mutex> lk(mu);
            if (!joy_enabled) {
                std::cout << "err joy only in pose" << std::endl;   // outside it, sticks walk
                continue;
            }
            for (int i = 0; i < 4; ++i) joy_v[i] = clamp(v[i], 1.0f);
            robot.joy(joy_v[0], joy_v[1], joy_v[2], joy_v[3]);
            joy_live = true;
            last_joy_ms = now_ms();
            std::cout << "ok joy " << joy_v[0] << "," << joy_v[1] << "," << joy_v[2] << ","
                      << joy_v[3] << std::endl;
            continue;
        }

        auto it = robot.verbs.find(verb);
        if (it == robot.verbs.end()) {
            std::cout << "err unknown verb" << std::endl;   // never reaches the robot
            continue;
        }
        int32_t r;
        {
            std::lock_guard<std::mutex> lk(mu);
            // Any verb ends a joystick session first (rule 5); a joy_on verb starts one after.
            joy_zero();
            joy_enabled = false;
            r = it->second();
            if (r == 0 && robot.joy_on.count(verb)) joy_enabled = true;
        }
        if (robot.stops_motion.count(verb)) moving.store(false);
        std::cout << (r == 0 ? "ok " : "err ") << verb << " " << r << std::endl;
    }

    // stdin closed: the HTTP layer is gone. Never leave a movement latched.
    std::cerr << "[" << tag << "] stdin closed — stopping the robot" << std::endl;
    {
        std::lock_guard<std::mutex> lk(mu);
        joy_zero();
        robot.stop_move();
    }
    running.store(false);
    deadman.join();
    return 0;
}

}  // namespace sender
