// g1_joint_sender — moves single arm joints of the G1 for the app's Joints page, through the
// robot's rt/arm_sdk topic. The rules are NOT in this file: they are in joint_ctl_core.hpp
// (read that first), which this only connects to DDS.
//
// Runs on the G1's PC2, spawned by relay_server.py when the page takes control (POST
// /joints/control {"op": "take"}) and alive for exactly ONE session: after a release (asked
// for, or forced by a rule) the weight is back at 0 and the process exits. A new `take` starts
// a new process, so no state from an earlier session can leak into the next one.
//
// WHAT IT TOUCHES
//   reads    rt/lowstate          the joints' measured positions (where control starts)
//            the loco service     the robot's FSM, every FSM_POLL_MS (GetFsmId, an RPC)
//            the BrainCo hands    finger positions over Modbus (src/brainco_modbus.hpp), 50 Hz
//   writes   rt/arm_sdk           only between `take` and the end of the release ramp
//            the BrainCo hands    finger targets (src/brainco_command.hpp: registers 1022-1033
//                                 only), only when the page moves a finger, inside a session
// It never writes rt/lowcmd: PC1 keeps balancing the robot the whole time.
//
// Configuration, all optional (relay.env):
//   JOINT_CTL_ALLOW     joints that accept a target, comma-separated  (every arm joint + waist yaw)
//   JOINT_CTL_FSM_OK    FSM ids control may start and stay in         (500,501)
//   JOINT_CTL_MAX_VEL   rad/s (capped at 1.0)                         (0.5)
//   JOINT_CTL_MAX_ACC   rad/s² (capped at 10)                         (2.0)
//   JOINT_CTL_STALL_FRAC  share of the rated torque that counts as pushing (0.3, 0.1-0.8)
//   JOINT_CTL_STALL_MS    ... held this long                          (150, 50-1000)
//   JOINT_CTL_EXCURSION rad from where control started; 0 = off       (0, the full URDF range)
//   JOINT_CTL_DEADMAN_MS / JOINT_CTL_RELEASE_MS                        (1000 / 5000)
//   HAND_SPEED          finger speed, of the hand's 1..1000 (capped at 600)  (300)
//   HANDS               0 = leave the hands alone                     (1)
// The caps are here and not only in relay.env so that a typo cannot make an arm fast.

#include "brainco_command.hpp"
#include "joint_ctl_core.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <atomic>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using Clock = std::chrono::steady_clock;

namespace {

// 200 Hz. Unitree's example runs at 50 Hz, and at 50 Hz each command is a visible step of the
// arm (felt as jerks, 2026-10-15); arm_sdk takes any rate, the robot's own loop is far faster.
constexpr float CONTROL_DT = 0.005f;
constexpr long FSM_POLL_MS = 250;

const char* env_s(const char* k, const char* d) {
    const char* v = getenv(k);
    return (v && *v) ? v : d;
}

float env_f(const char* k, float d, float lo, float hi) {
    const char* v = getenv(k);
    float x = (v && *v) ? strtof(v, nullptr) : d;
    if (!std::isfinite(x)) x = d;
    return std::min(hi, std::max(lo, x));
}

template <class Out, class F>
void split_csv(const std::string& s, F parse, Out& out) {
    std::string item;
    std::istringstream is(s);
    while (std::getline(is, item, ','))
        if (!item.empty()) out.insert(parse(item));
}

// Replies (reader thread) and events (control loop) share stdout: one whole line at a time.
std::mutex out_mu;
void say(const std::string& line) {
    std::lock_guard<std::mutex> lk(out_mu);
    std::cout << line << std::endl;
}

long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(Clock::now().time_since_epoch()).count();
}

struct Shared {
    std::mutex mu;                  // guards everything below AND the controller
    LowState low;
    long long low_at = -1;
    int fsm = -1;
    long long fsm_at = -1;
    int hand_pos[2][6] = {};
    long long hand_at[2] = {-1, -1};
};

constexpr long HAND_FRESH_MS = 200;   // a hand read longer ago than this does not count

}  // namespace

int main() {
    const std::string iface = env_s("DDS_IFACE", "eth0");
    jointctl::Config cfg;
    const std::string allow_env = env_s("JOINT_CTL_ALLOW", "");
    if (!allow_env.empty()) {           // unset = every movable joint (the Config default)
        cfg.allowed.clear();
        split_csv(allow_env, [](const std::string& s) { return s; }, cfg.allowed);
    }
    for (const auto& name : cfg.allowed) {
        if (jointctl::index_of(name) < 0 || jointctl::is_locked(name)) {
            std::cerr << "[g1-joint-sender] unknown joint in JOINT_CTL_ALLOW: " << name << std::endl;
            return 2;
        }
    }
    cfg.fsm_ok.clear();
    split_csv(env_s("JOINT_CTL_FSM_OK", "500,501"),
              [](const std::string& s) { return atoi(s.c_str()); }, cfg.fsm_ok);
    cfg.max_vel = env_f("JOINT_CTL_MAX_VEL", 0.5f, 0.05f, 1.0f);
    cfg.max_acc = env_f("JOINT_CTL_MAX_ACC", 2.0f, 0.2f, 10.0f);
    cfg.stall_frac = env_f("JOINT_CTL_STALL_FRAC", 0.3f, 0.1f, 0.8f);
    cfg.stall_ms = static_cast<long>(env_f("JOINT_CTL_STALL_MS", 150, 50, 1000));
    cfg.max_excursion = env_f("JOINT_CTL_EXCURSION", 0.0f, 0.0f, 6.3f);
    cfg.deadman_ms = static_cast<long>(env_f("JOINT_CTL_DEADMAN_MS", 1000, 200, 5000));
    cfg.release_ms = static_cast<long>(env_f("JOINT_CTL_RELEASE_MS", 5000, 500, 30000));
    cfg.hand_speed = static_cast<int>(env_f("HAND_SPEED", 300, 1, 600));

    std::cerr << "[g1-joint-sender] iface=" << iface << " allow="
              << (allow_env.empty() ? std::string("all") : allow_env)
              << " max_vel=" << cfg.max_vel << " max_acc=" << cfg.max_acc
              << " stall=" << cfg.stall_frac << "/" << cfg.stall_ms << "ms excursion="
              << cfg.max_excursion << " deadman=" << cfg.deadman_ms << "/" << cfg.release_ms
              << "ms" << std::endl;

    Shared sh;
    jointctl::Controller ctl(cfg);
    ChannelPublisherPtr<LowCmd> pub;
    ChannelSubscriberPtr<LowState> sub;
    std::shared_ptr<g1::LocoClient> loco;
    // Same reason as sender_core.hpp: a wrong DDS_IFACE is an uncaught DdsException otherwise.
    try {
        ChannelFactory::Instance()->Init(0, iface);
        pub.reset(new ChannelPublisher<LowCmd>("rt/arm_sdk"));
        pub->InitChannel();
        sub.reset(new ChannelSubscriber<LowState>("rt/lowstate"));
        sub->InitChannel([&sh](const void* msg) {
            std::lock_guard<std::mutex> lk(sh.mu);
            sh.low = *static_cast<const LowState*>(msg);
            sh.low_at = now_ms();
        }, 1);
        // Built after Init, like every SDK client (see sender::Robot).
        loco = std::make_shared<g1::LocoClient>();
        loco->SetTimeout(1.0f);
        loco->Init();
    } catch (const std::exception& e) {
        std::cerr << "[g1-joint-sender] cannot init DDS on '" << iface << "': " << e.what()
                  << std::endl;
        return 2;
    }

    std::atomic<bool> running{true};
    std::thread fsm_poll([&] {
        while (running.load()) {
            int fsm = -1;
            const bool ok = loco->GetFsmId(fsm) == 0;
            {
                std::lock_guard<std::mutex> lk(sh.mu);
                if (ok) {
                    sh.fsm = fsm;
                    sh.fsm_at = now_ms();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(FSM_POLL_MS));
        }
    });

    auto inputs = [&sh]() {           // caller holds sh.mu
        jointctl::Inputs in;
        const long long t = now_ms();
        in.state_age_ms = sh.low_at < 0 ? -1 : static_cast<long>(t - sh.low_at);
        if (sh.low_at >= 0)
            for (int i = 0; i < jointctl::NBODY; ++i) {
                const auto& m = sh.low.motor_state()[i];
                in.q[i] = m.q();
                in.tau[i] = m.tau_est();
                in.temp[i] = std::max<int>(m.temperature()[0], m.temperature()[1]);
            }
        if (sh.low_at >= 0)
            for (int k = 0; k < 3; ++k) in.rpy[k] = sh.low.imu_state().rpy()[k];
        in.fsm = sh.fsm;
        in.fsm_age_ms = sh.fsm_at < 0 ? -1 : static_cast<long>(t - sh.fsm_at);
        for (int h = 0; h < 2; ++h) {
            in.hand_ok[h] = sh.hand_at[h] >= 0 && t - sh.hand_at[h] < HAND_FRESH_MS;
            for (int f = 0; f < 6; ++f) in.hand_pos[h][f] = sh.hand_pos[h][f];
        }
        return in;
    };

    // The hands: found once, then read (and, inside a session, written) by a thread of their own
    // so a slow or missing hand never delays the arms' 200 Hz loop.
    std::unique_ptr<brainco::Port> hand_port[2];
    if (std::string(env_s("HANDS", "1")) != "0") {
        hand_port[0] = brainco::find_hand(brainco::LEFT_ID);
        hand_port[1] = brainco::find_hand(brainco::RIGHT_ID,
                                          hand_port[0] ? hand_port[0]->path() : "");
        for (int h = 0; h < 2; ++h)
            std::cerr << "[g1-joint-sender] " << (h ? "right" : "left") << " hand: "
                      << (hand_port[h] ? hand_port[h]->path() : std::string("not found"))
                      << std::endl;
    }
    const uint8_t hand_id[2] = {brainco::LEFT_ID, brainco::RIGHT_ID};
    std::thread hands_thread([&] {
        while (running.load()) {
            const auto t0 = Clock::now();
            for (int h = 0; h < 2; ++h) {
                if (!hand_port[h]) continue;
                uint16_t regs[brainco::STATUS_COUNT];
                if (hand_port[h]->read(hand_id[h], brainco::STATUS_ADDR, brainco::STATUS_COUNT,
                                       regs)) {
                    const auto st = brainco::decode_status(regs);
                    std::lock_guard<std::mutex> lk(sh.mu);
                    for (int f = 0; f < 6; ++f) sh.hand_pos[h][f] = st.pos[f];
                    sh.hand_at[h] = now_ms();
                }
                int pos[6], speed[6];
                bool due;
                {
                    std::lock_guard<std::mutex> lk(sh.mu);
                    due = ctl.hand_due(h, now_ms(), pos, speed);
                }
                if (due && brainco::write_targets(*hand_port[h], hand_id[h], pos, speed)) {
                    std::lock_guard<std::mutex> lk(sh.mu);
                    ctl.hand_written(h, now_ms(), pos);
                }
            }
            std::this_thread::sleep_until(t0 + std::chrono::milliseconds(20));
        }
    });

    // `take` is the first thing the relay sends, right after spawning us: answer it with a
    // real picture of the robot, not "FSM unknown" because the first RPC is still in flight.
    for (int k = 0; k < 75; ++k) {
        {
            std::lock_guard<std::mutex> lk(sh.mu);
            const bool hands_read = (!hand_port[0] || sh.hand_at[0] >= 0) &&
                                    (!hand_port[1] || sh.hand_at[1] >= 0);
            if (sh.low_at >= 0 && sh.fsm_at >= 0 && hands_read) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::thread reader([&] {
        std::string line;
        while (std::getline(std::cin, line)) {
            std::string reply;
            {
                std::lock_guard<std::mutex> lk(sh.mu);
                reply = ctl.handle(line, now_ms(), inputs());
            }
            say(reply);
        }
        std::lock_guard<std::mutex> lk(sh.mu);
        ctl.on_eof();
        std::cerr << "[g1-joint-sender] stdin closed — releasing" << std::endl;
    });
    reader.detach();                  // getline cannot be interrupted; the process exit ends it

    LowCmd msg;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<float>(CONTROL_DT));
    auto next = Clock::now();
    for (;;) {
        next += period;
        std::vector<std::string> events;
        jointctl::Output out;
        bool done;
        {
            std::lock_guard<std::mutex> lk(sh.mu);
            out = ctl.tick(now_ms(), CONTROL_DT, inputs(), events);
            done = ctl.finished();
        }
        if (out.publish) {
            for (int i = 0; i < jointctl::N; ++i) {
                auto& m = msg.motor_cmd()[jointctl::FIRST + i];
                m.q(out.q[i]);
                m.dq(0.0f);
                m.kp(out.kp);
                m.kd(out.kd);
                m.tau(0.0f);
            }
            msg.motor_cmd()[jointctl::WEIGHT_INDEX].q(out.weight);
            pub->Write(msg);
        }
        for (const auto& e : events) say("ev " + e);
        if (done) break;
        std::this_thread::sleep_until(next);
        if (Clock::now() - next > std::chrono::seconds(1)) next = Clock::now();
    }
    running.store(false);
    fsm_poll.join();
    hands_thread.join();
    std::cerr << "[g1-joint-sender] released, exiting" << std::endl;
    // _Exit, not return: the reader thread is still blocked reading stdin, holding stdin's
    // stdio lock, and a normal exit() flushes every stdio stream — it would wait on that lock
    // forever. Seen 2026-10-14: the process said "exiting" and never did, so the relay kept
    // the session open and refused the next "Take control".
    std::cout.flush();
    fflush(stdout);
    std::_Exit(0);
}
