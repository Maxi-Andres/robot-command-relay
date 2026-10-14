// g1_joint_bridge — the G1's joint state, for the app's Joints page. READ-ONLY for now.
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
//   rt/dex3/{left,right}/state   the two Dex3-1 hands, 7 motors each (both publish on this
//                        robot, seen 2026-10-13).
//
// WHAT IT DOES NOT DO: publish anything. There is no publisher header in this file, so this
// binary cannot move the robot. Step J2 of .claude/roadmap/G1-CONTROL-ARTICULAR.md adds the
// arm_sdk writer, and with it the safety envelope (limits, speed cap, weight ramp, dead-man) —
// that is when this file stops being read-only, and the header above changes with it.
//
// OUTPUT, one line per tick (numbers only, so no JSON string escaping is ever needed):
//   {"t":<unix s>,"age_ms":<ms since the last lowstate, -1 = never>,"mode_machine":6,
//    "imu":[roll,pitch,yaw],"q":[29],"dq":[29],"tau":[29],"temp":[29],
//    "left":{"age_ms":..,"q":[7],"tau":[7],"temp":[7]},"right":{...}}
// Radians, rad/s, N·m, °C. `temp` is the hotter of a motor's two sensors (the telemetry
// reader's rule). A hand that never published is "left":null.
//
// EOF ON STDIN EXITS: the relay holds our stdin, so if it dies we die with it instead of
// leaving an orphan DDS participant on the robot.

#include <unitree/idl/hg/HandState_.hpp>
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
#include <thread>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;
using HandState = unitree_hg::msg::dds_::HandState_;
using MotorState = unitree_hg::msg::dds_::MotorState_;

namespace {

constexpr int NBODY = 29;   // motor_state[0..28]; 29-34 are always zero on this robot
constexpr int NHAND = 7;    // Dex3-1: thumb 0-2, middle 0-1, index 0-1 (order: g1Joints.ts)

using Clock = std::chrono::steady_clock;

struct Latest {
    std::mutex mu;
    LowState low;
    HandState hand[2];
    Clock::time_point at_low{}, at_hand[2]{};
    bool have_low = false, have_hand[2] = {false, false};
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

void hand_json(std::string& s, const char* key, const HandState& h, bool have,
               Clock::time_point at, Clock::time_point now) {
    s += '"'; s += key; s += "\":";
    // A hand message with fewer motors than expected is treated as absent, never indexed past.
    if (!have || static_cast<int>(h.motor_state().size()) < NHAND) {
        s += "null";
        return;
    }
    s += "{\"age_ms\":" + std::to_string(age_ms(at, now)) + ',';
    const auto& m = h.motor_state();
    arr(s, "q", m, NHAND, [](const MotorState& x) { return x.q(); }, 4);
    s += ',';
    arr(s, "tau", m, NHAND, [](const MotorState& x) { return x.tau_est(); }, 3);
    s += ',';
    arr(s, "temp", m, NHAND, [](const MotorState& x) { return motor_temp(x); }, 0);
    s += '}';
}

std::string build_line() {
    LowState low;
    HandState hand[2];
    Clock::time_point at_low, at_hand[2];
    bool have_low, have_hand[2];
    {
        std::lock_guard<std::mutex> lk(g.mu);
        low = g.low;
        hand[0] = g.hand[0];
        hand[1] = g.hand[1];
        at_low = g.at_low;
        at_hand[0] = g.at_hand[0];
        at_hand[1] = g.at_hand[1];
        have_low = g.have_low;
        have_hand[0] = g.have_hand[0];
        have_hand[1] = g.have_hand[1];
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
    s += ',';
    hand_json(s, "left", hand[0], have_hand[0], at_hand[0], now);
    s += ',';
    hand_json(s, "right", hand[1], have_hand[1], at_hand[1], now);
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

    ChannelSubscriberPtr<HandState> sub_hand[2] = {
        ChannelSubscriberPtr<HandState>(new ChannelSubscriber<HandState>("rt/dex3/left/state")),
        ChannelSubscriberPtr<HandState>(new ChannelSubscriber<HandState>("rt/dex3/right/state"))};
    for (int side = 0; side < 2; ++side) {
        sub_hand[side]->InitChannel([side](const void* msg) {
            std::lock_guard<std::mutex> lk(g.mu);
            g.hand[side] = *static_cast<const HandState*>(msg);
            g.at_hand[side] = Clock::now();
            g.have_hand[side] = true;
        }, 1);
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
    return 0;
}
