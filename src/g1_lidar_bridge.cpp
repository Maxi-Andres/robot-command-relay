// g1_lidar_bridge — what is around the G1, from its Mid-360, as a small obstacle map for the
// relay's walk to a point (stop before hitting something) and the app's Navigation page.
// READ-ONLY: it listens to the lidar and to the robot's odometry, and writes nothing to either.
//
// Runs on PC2, spawned by relay_server.py while someone needs it (GET /lidar/stream, or a walk
// running), stopped once nobody does. One JSON line per tick on stdout; the relay keeps the newest.
//
// WHERE THE POINTS COME FROM (2026-11-05, tools/livox_query.py + livox_listen.py on the robot):
// the Mid-360 at 192.168.123.120 sends to PC1 unicast; tools/livox_set_host.py points it at the
// bus broadcast 192.168.123.255 so PC2 receives the same packets (PC1 too). Measured on PC2:
// 200 000 points/s, data type 1 (int32 mm), 2.7 MB/s, and its IMU at 200 Hz.
//
// THE MOUNT. The URDF puts mid360_link upright on the torso; the lidar's own accelerometer says
// otherwise (acc -0.50, +0.04, -0.88 g with the robot squatting): it is UPSIDE DOWN and tilted.
// So the points are levelled with the lidar's own gravity, not with the URDF:
//   1. R0 = the mount: a roll of LIDAR_MOUNT_ROLL_DEG (default 180, upside down) then a yaw of
//      LIDAR_MOUNT_YAW_DEG (default 0: the lidar's x is the robot's forward). TO VERIFY ON THE
//      ROBOT: a box straight ahead must come out at +x. If it comes out behind, set the yaw to 180.
//   2. the gravity measured (low-passed), seen through R0, is turned to straight up by the
//      smallest rotation — that removes the tilt (the mount's and the body's) and keeps heading.
// Then the points are placed in the odometry frame with the pose of rt/state_estimator/odom_torso
// at the moment they arrive, so a turning robot does not smear its own map.
//
// WHAT IS AN OBSTACLE: a point between FLOOR + LIDAR_MIN_H (10 cm) and FLOOR + LIDAR_MAX_H
// (1.5 m, the robot is 1.32 m), from LIDAR_SELF_M (0.35 m, its own arms and body) to
// LIDAR_RANGE_M (6 m) away. The floor is found in the points themselves: the densest low layer.
// Kept for LIDAR_WINDOW_MS (300 ms = 3 lidar frames): something that moves away is gone soon.
//
// OUTPUT, one line per tick at LIDAR_RATE_HZ (5):
//   {"t":<unix s>,"age_ms":<ms since the last point packet, -1 = never>,"pts_s":<points/s>,
//    "floor":<m under the lidar>,"tilt_deg":<lidar tilt off vertical>,"odom":0|1,
//    "pose":[x,y,yaw],"res":0.05,"cells":[[ix,iy],...]}
// cells: occupied 5 cm squares in the odometry frame (x = ix*res), within LIDAR_RANGE_M of the
// robot. Without odometry ("odom":0) they are in the robot's own levelled frame and pose is 0.
//
// EOF ON STDIN EXITS, like every bridge the relay runs.

#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace unitree::robot;
using Odometry = nav_msgs::msg::dds_::Odometry_;
using Clock = std::chrono::steady_clock;

namespace {

double env_d(const char* k, double d) {
    const char* v = getenv(k);
    return (v && *v) ? atof(v) : d;
}

double unix_s() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

long long mono_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(Clock::now().time_since_epoch()).count();
}

using Vec = std::array<double, 3>;
using Mat = std::array<double, 9>;  // row-major

Vec mul(const Mat& m, const Vec& v) {
    return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}

Mat mul(const Mat& a, const Mat& b) {
    Mat r{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) r[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
    return r;
}

Mat rot_x(double a) { return {1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a)}; }
Mat rot_z(double a) { return {cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1}; }

// The smallest rotation taking unit vector u to +z (Rodrigues). Keeps the heading.
Mat level_from(const Vec& u) {
    const double c = u[2];                       // cos of the angle
    Vec axis = {u[1], -u[0], 0};                 // u x z
    const double s = std::hypot(axis[0], axis[1]);
    if (s < 1e-9) return c > 0 ? Mat{1, 0, 0, 0, 1, 0, 0, 0, 1} : rot_x(M_PI);
    axis = {axis[0] / s, axis[1] / s, 0};
    const double t = 1 - c, x = axis[0], y = axis[1];
    return {c + x * x * t, x * y * t, y * s, x * y * t, c + y * y * t, -x * s, -y * s, x * s, c};
}

struct Pose {
    double x = 0, y = 0, yaw = 0;
    bool valid = false;
};

struct State {
    std::mutex mu;
    Vec acc_lp{0, 0, 0};
    bool have_acc = false;
    Pose pose;
    long long pose_ms = 0;
    long long last_pkt_ms = -1;
    long long points = 0;
    double floor = -1.2;                         // m under the lidar, until measured
    // cell key -> last time seen (ms)
    std::unordered_map<long long, long long> cells;
};
State g;

long long key(int ix, int iy) { return ((long long)ix << 32) ^ (unsigned int)iy; }

int open_udp(int port) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1, buf = 4 * 1024 * 1024;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &one, sizeof one);
    setsockopt(s, SOL_SOCKET, SO_RCVBUF, &buf, sizeof buf);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (sockaddr*)&a, sizeof a) < 0) {
        perror("bind");
        exit(1);
    }
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string iface = argc > 1 ? argv[1] : (getenv("DDS_IFACE") ? getenv("DDS_IFACE") : "eth0");
    const double res = 0.05;
    const double min_h = env_d("LIDAR_MIN_H", 0.10), max_h = env_d("LIDAR_MAX_H", 1.50);
    const double self_m = env_d("LIDAR_SELF_M", 0.35), range_m = env_d("LIDAR_RANGE_M", 6.0);
    const long long window_ms = (long long)env_d("LIDAR_WINDOW_MS", 300);
    double hz = env_d("LIDAR_RATE_HZ", 5);
    if (!(hz >= 1 && hz <= 20)) hz = 5;
    const Mat R0 = mul(rot_z(env_d("LIDAR_MOUNT_YAW_DEG", 0) * M_PI / 180),
                       rot_x(env_d("LIDAR_MOUNT_ROLL_DEG", 180) * M_PI / 180));
    std::cerr << "[lidar-bridge] iface=" << iface << " obstacles " << min_h << ".." << max_h
              << " m over the floor, " << self_m << ".." << range_m << " m away" << std::endl;

    ChannelFactory::Instance()->Init(0, iface);
    ChannelSubscriberPtr<Odometry> odom(new ChannelSubscriber<Odometry>(
        getenv("ODOM_TOPIC") ? getenv("ODOM_TOPIC") : "rt/state_estimator/odom_torso"));
    odom->InitChannel([](const void* msg) {
        const auto* o = (const Odometry*)msg;
        const auto& p = o->pose().pose().position();
        const auto& q = o->pose().pose().orientation();
        std::lock_guard<std::mutex> lk(g.mu);
        g.pose.x = p.x();
        g.pose.y = p.y();
        g.pose.yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                                1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        g.pose.valid = true;
        g.pose_ms = mono_ms();
    }, 1);

    const int sp = open_udp(56301), si = open_udp(56401);
    std::atomic<bool> done{false};
    std::thread eof([&] {
        std::string line;
        while (std::getline(std::cin, line)) {
        }
        done = true;
    });

    std::thread rx([&] {
        std::vector<uint8_t> buf(2048);
        pollfd fds[2] = {{sp, POLLIN, 0}, {si, POLLIN, 0}};
        // The floor: a histogram of levelled z, 2 cm bins from -2.5 m to 0, refreshed per frame.
        std::array<int, 125> hist{};
        int hist_n = 0;
        while (!done) {
            if (poll(fds, 2, 200) <= 0) continue;
            for (int f = 0; f < 2; f++) {
                if (!(fds[f].revents & POLLIN)) continue;
                const ssize_t n = recv(fds[f].fd, buf.data(), buf.size(), 0);
                if (n < 36) continue;
                const uint16_t dots = buf[5] | (buf[6] << 8);
                const uint8_t type = buf[10];
                if (type == 0 && n >= 36 + 24) {            // IMU: acc in g at 24..36
                    float a[3];
                    memcpy(a, buf.data() + 36 + 12, 12);
                    std::lock_guard<std::mutex> lk(g.mu);
                    for (int i = 0; i < 3; i++)
                        g.acc_lp[i] = g.have_acc ? 0.98 * g.acc_lp[i] + 0.02 * a[i] : a[i];
                    g.have_acc = true;
                    continue;
                }
                if (type != 1 && type != 2) continue;
                const int step = type == 1 ? 14 : 8;
                if (36 + (ssize_t)dots * step > n) continue;
                Mat R;
                Pose pose;
                double floor;
                {
                    std::lock_guard<std::mutex> lk(g.mu);
                    if (!g.have_acc) continue;           // cannot level without gravity yet
                    Vec u = mul(R0, g.acc_lp);
                    const double m = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
                    if (m < 0.5) continue;
                    u = {u[0] / m, u[1] / m, u[2] / m};
                    R = mul(level_from(u), R0);
                    pose = g.pose;
                    if (mono_ms() - g.pose_ms > 500) pose = Pose{};
                    floor = g.floor;
                    g.last_pkt_ms = mono_ms();
                    g.points += dots;
                }
                const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
                const long long now = mono_ms();
                std::vector<long long> hit;
                hit.reserve(dots);
                for (int k = 0; k < dots; k++) {
                    const uint8_t* p = buf.data() + 36 + k * step;
                    double x, y, z;
                    if (type == 1) {
                        int32_t v[3];
                        memcpy(v, p, 12);
                        x = v[0] * 0.001, y = v[1] * 0.001, z = v[2] * 0.001;
                    } else {
                        int16_t v[3];
                        memcpy(v, p, 6);
                        x = v[0] * 0.01, y = v[1] * 0.01, z = v[2] * 0.01;
                    }
                    if (x == 0 && y == 0 && z == 0) continue;
                    const Vec q = mul(R, Vec{x, y, z});
                    const double r = std::hypot(q[0], q[1]);
                    if (r < self_m || r > range_m) continue;
                    if (r > 0.6 && r < 4.0 && q[2] > -2.5 && q[2] < 0) {
                        hist[std::min(124, (int)((q[2] + 2.5) / 0.02))]++;
                        hist_n++;
                    }
                    if (q[2] < floor + min_h || q[2] > floor + max_h) continue;
                    const double wx = pose.x + c * q[0] - s * q[1];
                    const double wy = pose.y + s * q[0] + c * q[1];
                    hit.push_back(key((int)std::lround(wx / res), (int)std::lround(wy / res)));
                }
                // About one lidar frame (10 000 points): place the floor at the densest low layer.
                if (hist_n > 10000) {
                    int best = 0;
                    for (int b = 1; b < 125; b++)
                        if (hist[b] > hist[best]) best = b;
                    const double z = -2.5 + (best + 0.5) * 0.02;
                    std::lock_guard<std::mutex> lk(g.mu);
                    g.floor = 0.8 * g.floor + 0.2 * z;
                    hist.fill(0);
                    hist_n = 0;
                }
                std::lock_guard<std::mutex> lk(g.mu);
                for (long long k : hit) g.cells[k] = now;
            }
        }
    });

    const auto period = std::chrono::duration<double>(1.0 / hz);
    auto next = Clock::now();
    long long last_points = 0;
    auto last_count = Clock::now();
    while (!done) {
        next += std::chrono::duration_cast<Clock::duration>(period);
        std::this_thread::sleep_until(next);
        std::string cells;
        long long age = -1, pts = 0;
        double floor, tilt = 0;
        Pose pose;
        {
            std::lock_guard<std::mutex> lk(g.mu);
            const long long now = mono_ms();
            if (g.last_pkt_ms >= 0) age = now - g.last_pkt_ms;
            pose = g.pose;
            if (now - g.pose_ms > 500) pose = Pose{};
            floor = g.floor;
            if (g.have_acc) {
                const Vec u = mul(R0, g.acc_lp);
                const double m = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
                if (m > 0.5) tilt = std::acos(std::max(-1.0, std::min(1.0, u[2] / m))) * 180 / M_PI;
            }
            const double secs = std::chrono::duration<double>(Clock::now() - last_count).count();
            pts = secs > 0 ? (long long)((g.points - last_points) / secs) : 0;
            last_points = g.points;
            last_count = Clock::now();
            int n = 0;
            for (auto it = g.cells.begin(); it != g.cells.end();) {
                const int ix = (int)(it->first >> 32), iy = (int)(int32_t)(it->first & 0xffffffff);
                if (now - it->second > window_ms ||
                    std::hypot(ix * res - pose.x, iy * res - pose.y) > range_m) {
                    it = g.cells.erase(it);
                    continue;
                }
                if (n++ < 8000) {
                    cells += cells.empty() ? "" : ",";
                    cells += "[" + std::to_string(ix) + "," + std::to_string(iy) + "]";
                }
                ++it;
            }
        }
        printf("{\"t\":%.3f,\"age_ms\":%lld,\"pts_s\":%lld,\"floor\":%.3f,\"tilt_deg\":%.1f,"
               "\"odom\":%d,\"pose\":[%.4f,%.4f,%.5f],\"res\":%.2f,\"cells\":[%s]}\n",
               unix_s(), age, pts, floor, tilt, pose.valid ? 1 : 0, pose.x, pose.y, pose.yaw,
               res, cells.c_str());
        fflush(stdout);
    }
    rx.join();
    eof.join();
    close(sp);
    close(si);
    return 0;
}
