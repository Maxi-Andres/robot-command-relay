// g1_odom_bridge — where the G1 thinks it is, for the app's Navigation page and for the relay's
// own "walk to a point" (POST /walk/control). READ-ONLY.
//
// Runs on PC2 next to the robot's DDS, spawned by relay_server.py while someone needs the pose
// (a viewer of GET /odom/stream, or a walk running), and stopped again once nobody does. It prints
// one JSON line per tick on stdout; the relay keeps only the newest line.
//
// WHAT IT READS
//   rt/state_estimator/odom_torso   nav_msgs/Odometry, frame "odom" -> child "torso", 51.6 Hz,
//                                    published by the robot itself with no mapping running
//                                    (tools/g1_nav_probe, 2026-11-05). Leg + IMU odometry:
//                                    1.978 m reported for 2 m walked, 0 m drift standing 60 s.
//                                    Its origin is wherever the robot booted: it is NOT a frame
//                                    of the room. ODOM_TOPIC=... reads another Odometry topic
//                                    (odom_pelvis, or the lidar-fused fusion_odom once it talks).
//
// WHAT IT DOES NOT DO: move anything. Its only request is GetFsmId, a query.
//
// OUTPUT, one line per tick at ODOM_RATE_HZ (numbers only, no JSON string escaping needed):
//   {"t":<unix s>,"age_ms":<ms since the last odometry, -1 = never>,
//    "x":m,"y":m,"z":m,"yaw":rad,"vx":m/s,"vy":m/s,"wz":rad/s,"fsm":<locomotion mode, -1 unknown>}
// x/y/z/yaw in the odom frame; vx/vy/wz as the message carries them (its twist).
//
// EOF ON STDIN EXITS: the relay holds our stdin, so if it dies we die with it.

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using namespace unitree::robot;
using Odometry = nav_msgs::msg::dds_::Odometry_;
using Clock = std::chrono::steady_clock;

namespace {

struct Latest {
    std::mutex mu;
    Odometry odom;
    Clock::time_point at{};
    bool have = false;
    // The locomotion mode, ASKED of the loco service every 500 ms (GetFsmId, the same RPC the
    // joint sender trusts; rt/sportmodestate's fsm_id read 0 with the robot squatting,
    // 2026-11-05): 500/501 walk, 801/802 run, 706 squat, 1 damp... -1 unknown. The relay's walk
    // switches walk <-> run only from a locomotion mode, and never stands a squatting robot up.
    // Asking moves nothing: GetFsmId is a query.
    int fsm = -1;
    Clock::time_point fsm_at{};
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

}  // namespace

int main(int argc, char** argv) {
    const std::string iface = argc > 1 ? argv[1] : env_s("DDS_IFACE", "eth0");
    const std::string topic = env_s("ODOM_TOPIC", "rt/state_estimator/odom_torso");
    double hz = atof(env_s("ODOM_RATE_HZ", "20"));
    if (!(hz >= 1 && hz <= 50)) hz = 20;
    std::cerr << "[odom-bridge] iface=" << iface << " topic=" << topic << " rate=" << hz << "Hz"
              << std::endl;

    // Binding Cyclone to the interface is not optional: without it nothing arrives.
    ChannelFactory::Instance()->Init(0, iface);
    ChannelSubscriberPtr<Odometry> sub(new ChannelSubscriber<Odometry>(topic));
    sub->InitChannel([](const void* msg) {
        std::lock_guard<std::mutex> lk(g.mu);
        g.odom = *(const Odometry*)msg;
        g.at = Clock::now();
        g.have = true;
    }, 1);
    // Built after Init, like every SDK client (a client built earlier segfaults).
    auto loco = std::make_shared<g1::LocoClient>();
    loco->SetTimeout(1.0f);
    loco->Init();

    std::atomic<bool> done{false};
    std::thread fsm_poll([&] {
        while (!done) {
            int fsm = -1;
            if (loco->GetFsmId(fsm) == 0) {
                std::lock_guard<std::mutex> lk(g.mu);
                g.fsm = fsm;
                g.fsm_at = Clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    std::thread eof([&] {
        std::string line;
        while (std::getline(std::cin, line)) {
        }
        done = true;
    });

    const auto period = std::chrono::duration<double>(1.0 / hz);
    auto next = Clock::now();
    while (!done) {
        next += std::chrono::duration_cast<Clock::duration>(period);
        std::this_thread::sleep_until(next);
        Odometry o;
        long age = -1;
        int fsm = -1;
        {
            std::lock_guard<std::mutex> lk(g.mu);
            if (std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - g.fsm_at)
                    .count() < 2000)
                fsm = g.fsm;
            if (g.have) {
                o = g.odom;
                age = (long)std::chrono::duration_cast<std::chrono::milliseconds>(
                          Clock::now() - g.at).count();
            }
        }
        if (age < 0) {
            printf("{\"t\":%.3f,\"age_ms\":-1}\n", unix_s());
        } else {
            const auto& p = o.pose().pose().position();
            const auto& q = o.pose().pose().orientation();
            const auto& v = o.twist().twist().linear();
            const double yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                                          1 - 2 * (q.y() * q.y() + q.z() * q.z()));
            printf("{\"t\":%.3f,\"age_ms\":%ld,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"yaw\":%.5f,"
                   "\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f,\"fsm\":%d}\n",
                   unix_s(), age, p.x(), p.y(), p.z(), yaw, v.x(), v.y(),
                   o.twist().twist().angular().z(), fsm);
        }
        fflush(stdout);
    }
    eof.join();
    fsm_poll.join();
    return 0;
}
