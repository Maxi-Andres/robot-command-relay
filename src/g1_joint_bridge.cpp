// g1_joint_bridge — the G1's joint state, for the app's Joints page. READ-ONLY.
//
// Runs on PC2 next to the robot's DDS, spawned by relay_server.py while at least one viewer
// is watching (GET /joints/stream), and stopped again once nobody is. It prints one JSON line
// per tick on stdout; the relay keeps only the newest and hands it to every viewer.
//
// WHAT IT READS
//   rt/lowstate          the 29 body joints in the SDK's G1 29-DoF order, plus the pelvis IMU.
//                        1039 Hz on the robot (measured 2026-09-30); we keep the newest and
//                        print at JOINT_RATE_HZ. JOINT_STATE_TOPIC=rt/lf/lowstate (20 Hz) is the
//                        cheap alternative if PC2 is ever short of CPU.
//
//   the two hands        BrainCo Revo 2, over Modbus RTU on the FTDI board that PC2 sees as
//                        /dev/ttyUSB* (src/brainco_modbus.hpp: registers, ids, why). Each port
//                        is probed once for the left (126) and right (127) hand; then both are
//                        read at JOINT_RATE_HZ. HANDS=0 turns this off. Before 2026-10-15 this
//                        file read rt/dex3/*/state instead — the wrong hand (Dex3-1) entirely.
//
// WHAT IT DOES NOT DO: move anything. There is no DDS publisher in this file, and the hands'
// Modbus code can only build read requests (function 04). Moving the arms is g1_joint_sender's
// job (step J2-J3); moving the fingers will be a separate writer with its own limits (J4).
//
// OUTPUT, one line per tick (numbers only, so no JSON string escaping is ever needed):
//   {"t":<unix s>,"age_ms":<ms since the last lowstate, -1 = never>,"mode_machine":6,
//    "imu":[roll,pitch,yaw],"q":[29],"dq":[29],"tau":[29],"temp":[29],
//    "left":{"age_ms":..,"pos":[6],"speed":[6],"current":[6],"state":[6],"errors":n}|null,
//    "right":{...}|null}
// Radians, rad/s, N·m, °C. `temp` is the hotter of a motor's two sensors (the telemetry
// reader's rule). Hands in the hand's own units: pos 0 open .. 1000 closed, speed ±1000,
// current signed, state 0 idle / 1 running / 2 stalled / 3 turbo; finger order thumb flex,
// thumb aux, index, middle, ring, pinky. A hand that was never found is null.
//
// EOF ON STDIN EXITS: the relay holds our stdin, so if it dies we die with it instead of
// leaving an orphan DDS participant on the robot.

#include "brainco_modbus.hpp"

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <memory>
#include <thread>
#include <vector>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;
using MotorState = unitree_hg::msg::dds_::MotorState_;

namespace {

constexpr int NBODY = 29;   // motor_state[0..28]; 29-34 are always zero on this robot

using Clock = std::chrono::steady_clock;

struct Latest {
    std::mutex mu;
    LowState low;
    Clock::time_point at_low{};
    bool have_low = false;
};
Latest g;

const char* env_s(const char* k, const char* d) {
    const char* v = getenv(k);
    return (v && *v) ? v : d;
}

double unix_s() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

long age_ms(Clock::time_point at, Clock::time_point now) {
    return static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - at).count());
}

// Trailing zeros trimmed (0.1400 -> 0.14, 0.000 -> 0): at 30 lines a second per viewer, over
// the robot's own uplink, a quarter of the line was zeros.
void num(std::string& s, double v, int prec) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", prec, v);
    std::string t(buf);
    if (t.find('.') != std::string::npos) {
        while (t.back() == '0') t.pop_back();
        if (t.back() == '.') t.pop_back();
    }
    if (t == "-0") t = "0";
    s += t;
}

int motor_temp(const MotorState& m) {
    return std::max<int>(m.temperature()[0], m.temperature()[1]);
}

// "key":[a,b,...] over `n` motors, `get` picking the field.
template <class Motors, class Get>
void arr(std::string& s, const char* key, const Motors& motors, int n, Get get, int prec) {
    s += '"'; s += key; s += "\":[";
    for (int i = 0; i < n; ++i) {
        if (i) s += ',';
        num(s, get(motors[i]), prec);
    }
    s += ']';
}

// ---- the hands ----------------------------------------------------------------------------

struct Hand {
    const char* key;                 // "left" / "right" in the JSON
    uint8_t id;
    std::unique_ptr<brainco::Port> port;
    brainco::Status st{};
    Clock::time_point at{};
    bool have = false;
    long errors = 0;                 // failed reads since the last good one
};
std::mutex hands_mu;
Hand hands[2] = {{"left", brainco::LEFT_ID, nullptr, {}, {}, false, 0},
                 {"right", brainco::RIGHT_ID, nullptr, {}, {}, false, 0}};

// Each missing hand is looked for on every candidate port (brainco::find_hand: /dev/ttyUSB0-7,
// or HAND_PORTS), skipping the port the other hand already holds.
void find_hands() {
    for (int i = 0; i < 2; ++i) {
        std::string other;
        {
            std::lock_guard<std::mutex> lk(hands_mu);
            if (hands[i].port) continue;
            if (hands[1 - i].port) other = hands[1 - i].port->path();
        }
        auto port = brainco::find_hand(hands[i].id, other);
        if (!port) continue;
        fprintf(stderr, "[g1-joints] %s hand (id %d) on %s\n", hands[i].key, hands[i].id,
                port->path().c_str());
        std::lock_guard<std::mutex> lk(hands_mu);
        hands[i].port = std::move(port);
    }
}

void hands_loop(const std::atomic<bool>& running, double rate) {
    const auto period = std::chrono::duration<double>(1.0 / rate);
    auto last_scan = Clock::now() - std::chrono::seconds(10);
    while (running) {
        const auto t0 = Clock::now();
        bool missing = false;
        for (Hand& h : hands) {
            brainco::Port* port;
            {
                std::lock_guard<std::mutex> lk(hands_mu);
                port = h.port.get();
            }
            if (!port) { missing = true; continue; }
            uint16_t regs[brainco::STATUS_COUNT];
            const bool ok = port->read(h.id, brainco::STATUS_ADDR, brainco::STATUS_COUNT, regs);
            std::lock_guard<std::mutex> lk(hands_mu);
            if (ok) {
                h.st = brainco::decode_status(regs);
                h.at = Clock::now();
                h.have = true;
                h.errors = 0;
            } else if (++h.errors >= 60) {
                // ~2 s of silence: unplugged or re-enumerated. Look for it again.
                fprintf(stderr, "[g1-joints] %s hand stopped answering on %s\n", h.key,
                        h.port->path().c_str());
                h.port.reset();
                h.errors = 0;
            }
        }
        // A missing hand is looked for again every 5 s (the USB cable was found unplugged once).
        if (missing && Clock::now() - last_scan > std::chrono::seconds(5)) {
            find_hands();
            last_scan = Clock::now();
        }
        std::this_thread::sleep_until(t0 + std::chrono::duration_cast<Clock::duration>(period));
    }
}

void hand_json(std::string& s, const Hand& h, Clock::time_point now) {
    s += '"'; s += h.key; s += "\":";
    if (!h.have) {
        s += "null";
        return;
    }
    s += "{\"age_ms\":" + std::to_string(age_ms(h.at, now));
    auto arr = [&s](const char* key, const int* v) {
        s += ",\""; s += key; s += "\":[";
        for (int i = 0; i < brainco::FINGERS; ++i) {
            if (i) s += ',';
            s += std::to_string(v[i]);
        }
        s += ']';
    };
    arr("pos", h.st.pos);
    arr("speed", h.st.speed);
    arr("current", h.st.current);
    arr("state", h.st.state);
    s += ",\"errors\":" + std::to_string(h.errors) + '}';
}

std::string build_line() {
    LowState low;
    Clock::time_point at_low;
    bool have_low;
    {
        std::lock_guard<std::mutex> lk(g.mu);
        low = g.low;
        at_low = g.at_low;
        have_low = g.have_low;
    }
    const auto now = Clock::now();
    std::string s = "{\"t\":";
    num(s, unix_s(), 3);
    s += ",\"age_ms\":" + std::to_string(have_low ? age_ms(at_low, now) : -1);
    if (have_low) {
        s += ",\"mode_machine\":" + std::to_string(low.mode_machine()) + ',';
        const auto& rpy = low.imu_state().rpy();
        s += "\"imu\":[";
        num(s, rpy[0], 4); s += ','; num(s, rpy[1], 4); s += ','; num(s, rpy[2], 4);
        s += "],";
        const auto& m = low.motor_state();
        arr(s, "q", m, NBODY, [](const MotorState& x) { return x.q(); }, 4);
        s += ',';
        arr(s, "dq", m, NBODY, [](const MotorState& x) { return x.dq(); }, 3);
        s += ',';
        arr(s, "tau", m, NBODY, [](const MotorState& x) { return x.tau_est(); }, 3);
        s += ',';
        arr(s, "temp", m, NBODY, [](const MotorState& x) { return motor_temp(x); }, 0);
    }
    {
        std::lock_guard<std::mutex> lk(hands_mu);
        for (const Hand& h : hands) {
            s += ',';
            hand_json(s, h, now);
        }
    }
    s += '}';
    return s;
}

}  // namespace

int main() {
    const std::string iface = env_s("DDS_IFACE", "eth0");
    const std::string topic = env_s("JOINT_STATE_TOPIC", "rt/lowstate");
    double rate = atof(env_s("JOINT_RATE_HZ", "30"));
    rate = std::min(60.0, std::max(1.0, rate));   // a typo must not flood the relay or stall

    fprintf(stderr, "[g1-joints] iface=%s topic=%s rate=%.0f Hz (read-only)\n",
            iface.c_str(), topic.c_str(), rate);

    // Binding CycloneDDS to the interface is not optional: Init(0, iface) is what makes the SDK
    // receive anything (same finding as the telemetry reader).
    ChannelFactory::Instance()->Init(0, iface);

    ChannelSubscriberPtr<LowState> sub_low(new ChannelSubscriber<LowState>(topic));
    sub_low->InitChannel([](const void* msg) {
        std::lock_guard<std::mutex> lk(g.mu);
        g.low = *static_cast<const LowState*>(msg);
        g.at_low = Clock::now();
        g.have_low = true;
    }, 1);

    // The hands: a thread of their own, so a slow or missing hand never delays the body.
    std::atomic<bool> hands_on{env_s("HANDS", "1") != std::string("0")};
    std::thread hands_thread;
    if (hands_on) {
        find_hands();
        hands_thread = std::thread(hands_loop, std::cref(hands_on), rate);
    }

    std::atomic<bool> stdin_open{true};
    std::thread([&stdin_open] {
        std::string line;
        while (std::getline(std::cin, line)) {
        }
        stdin_open = false;
    }).detach();

    const auto period = std::chrono::duration<double>(1.0 / rate);
    auto next = Clock::now();
    while (stdin_open) {
        next += std::chrono::duration_cast<Clock::duration>(period);
        const std::string line = build_line();
        // fwrite to a closed pipe raises SIGPIPE and ends the process: the relay is gone, and
        // so is the reason to run.
        fwrite(line.data(), 1, line.size(), stdout);
        fputc('\n', stdout);
        fflush(stdout);
        std::this_thread::sleep_until(next);
        // After a long stall (machine suspended, debugger) do not burst to catch up.
        if (Clock::now() - next > std::chrono::seconds(1)) next = Clock::now();
    }
    fprintf(stderr, "[g1-joints] stdin closed, exiting\n");
    hands_on = false;
    if (hands_thread.joinable()) hands_thread.join();
    return 0;
}
