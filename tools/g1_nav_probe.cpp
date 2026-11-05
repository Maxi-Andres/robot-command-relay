// g1_nav_probe — what the G1 already gives us for a map: which DDS topics exist, which of them
// carry a point cloud, a pose, an odometry or a map, how often, how heavy, and in what frame.
// Read-only: it creates no publisher and moves nothing.
//
// The question it answers (roadmap G1.md §6.2 "Inventario: lo que falta", §6.8): the Mid-360 at
// 192.168.123.120 was never read; G1.md says odometry does not publish without Unitree's mapping,
// but rt/odommodestate was seen at 51 Hz on 2026-08-03. Whether we build SLAM ourselves (FAST-LIO
// on PC2) or use Unitree's depends on what this prints.
//
// Run it ON PC2 through tools/g1_nav_probe.sh: it builds this with the same flags as ../build.sh
// (the SDK's static library and its Cyclone), adds the network side, and saves a report. The
// binary alone:
//   tools/g1_nav_probe eth0 [discover_s=5] [measure_s=10]
//
// 1. DISCOVERY: every topic some writer on the bus publishes, with its type, how many writers,
//    and their reliability (a RELIABLE 200 KB point cloud over WiFi is a retransmit storm).
// 2. MEASURE: every topic whose type is one a map is made of (point cloud, odometry, pose, IMU,
//    occupancy grid, sport state) is read for measure_s with a BEST_EFFORT reader (it matches
//    both kinds of writer) — rate, payload size, and one sample in words.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <dds/dds.h>
#include <dds/dds.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/SportModeState_.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/OccupancyGrid_.hpp>
#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/PoseStamped_.hpp>

namespace {

double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

// ---------------------------------------------------------------------------------------------
// 1. Discovery, through Cyclone's built-in DCPSPublication topic (no types needed)
// ---------------------------------------------------------------------------------------------
struct Seen {
    std::string type;
    std::set<std::string> writers;  // GUIDs
    bool reliable = false, best_effort = false;
};

std::string guid_str(const dds_guid_t& g) {
    std::string s;
    for (unsigned char b : g.v) s += fmt("%02x", b);
    return s;
}

std::map<std::string, Seen> discover(double seconds) {
    std::map<std::string, Seen> out;
    // Attaches to domain 0 as ChannelFactory configured it (bound to the interface).
    dds_entity_t pp = dds_create_participant(0, nullptr, nullptr);
    if (pp < 0) {
        fprintf(stderr, "discovery: participant failed: %s\n", dds_strretcode(pp));
        return out;
    }
    dds_entity_t rd = dds_create_reader(pp, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr);
    if (rd < 0) {
        fprintf(stderr, "discovery: reader failed: %s\n", dds_strretcode(rd));
        dds_delete(pp);
        return out;
    }
    const double end = now_s() + seconds;
    void* samples[64] = {};
    dds_sample_info_t infos[64];
    while (now_s() < end) {
        const int n = dds_take(rd, samples, infos, 64, 64);
        for (int i = 0; i < n; i++) {
            if (!infos[i].valid_data) continue;
            auto* ep = static_cast<dds_builtintopic_endpoint_t*>(samples[i]);
            Seen& s = out[ep->topic_name];
            s.type = ep->type_name;
            s.writers.insert(guid_str(ep->key));
            dds_reliability_kind_t kind;
            dds_duration_t block;
            if (ep->qos && dds_qget_reliability(ep->qos, &kind, &block)) {
                (kind == DDS_RELIABILITY_RELIABLE ? s.reliable : s.best_effort) = true;
            }
        }
        if (n > 0) dds_return_loan(rd, samples, n);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    dds_delete(pp);
    return out;
}

bool interesting(const std::string& name, const std::string& type) {
    static const char* words[] = {"lidar", "livox", "cloud", "point", "odom", "slam", "lio",
                                  "map", "pose", "loc", "nav", "imu", "scan", "path", "grid",
                                  "height", "sport"};
    std::string s = name + " " + type;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    for (const char* w : words)
        if (s.find(w) != std::string::npos) return true;
    return false;
}

// ---------------------------------------------------------------------------------------------
// 2. Measuring the topics whose type we know
// ---------------------------------------------------------------------------------------------
struct Stat {
    std::string topic, type;
    long count = 0;
    double bytes = 0;
    double first = 0, last = 0;
    std::string sample;  // the first message, in words
};

std::string header_str(const std_msgs::msg::dds_::Header_& h) {
    return fmt("frame '%s', stamp %d.%09u", h.frame_id().c_str(), h.stamp().sec(),
               h.stamp().nanosec());
}

std::string yaw_of(double x, double y, double z, double w) {
    return fmt("yaw %.1f deg", std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 57.2958);
}

// Point cloud: the fields, and the range of the points if x/y/z are float32 (most drivers).
std::string describe(const sensor_msgs::msg::dds_::PointCloud2_& m) {
    std::string fields;
    int ox = -1, oy = -1, oz = -1;
    for (const auto& f : m.fields()) {
        fields += fmt("%s%s(t%u@%u)", fields.empty() ? "" : " ", f.name().c_str(), f.datatype(),
                      f.offset());
        if (f.datatype() == 7) {  // FLOAT32
            if (f.name() == "x") ox = f.offset();
            if (f.name() == "y") oy = f.offset();
            if (f.name() == "z") oz = f.offset();
        }
    }
    std::string s = fmt("%s | %ux%u points, point_step %u, dense %d | fields: %s",
                        header_str(m.header()).c_str(), m.width(), m.height(), m.point_step(),
                        (int)m.is_dense(), fields.c_str());
    const size_t step = m.point_step(), n = step ? m.data().size() / step : 0;
    if (ox >= 0 && oy >= 0 && oz >= 0 && n > 0) {
        std::vector<float> r;
        float zmin = 1e9f, zmax = -1e9f;
        for (size_t i = 0; i < n; i++) {
            float x, y, z;
            const uint8_t* p = m.data().data() + i * step;
            memcpy(&x, p + ox, 4);
            memcpy(&y, p + oy, 4);
            memcpy(&z, p + oz, 4);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            r.push_back(std::sqrt(x * x + y * y + z * z));
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
        if (!r.empty()) {
            std::sort(r.begin(), r.end());
            s += fmt(" | range m: min %.2f median %.2f max %.2f, z %.2f..%.2f", r.front(),
                     r[r.size() / 2], r.back(), zmin, zmax);
        }
    }
    return s;
}

std::string describe(const nav_msgs::msg::dds_::Odometry_& m) {
    const auto& p = m.pose().pose().position();
    const auto& q = m.pose().pose().orientation();
    const auto& v = m.twist().twist().linear();
    return fmt("%s -> child '%s' | pos %.3f %.3f %.3f, %s | v %.2f %.2f %.2f",
               header_str(m.header()).c_str(), m.child_frame_id().c_str(), p.x(), p.y(), p.z(),
               yaw_of(q.x(), q.y(), q.z(), q.w()).c_str(), v.x(), v.y(), v.z());
}

std::string describe(const geometry_msgs::msg::dds_::PoseStamped_& m) {
    const auto& p = m.pose().position();
    const auto& q = m.pose().orientation();
    return fmt("%s | pos %.3f %.3f %.3f, %s", header_str(m.header()).c_str(), p.x(), p.y(), p.z(),
               yaw_of(q.x(), q.y(), q.z(), q.w()).c_str());
}

std::string describe(const sensor_msgs::msg::dds_::Imu_& m) {
    const auto& a = m.linear_acceleration();
    return fmt("%s | accel %.2f %.2f %.2f", header_str(m.header()).c_str(), a.x(), a.y(), a.z());
}

std::string describe(const nav_msgs::msg::dds_::OccupancyGrid_& m) {
    const auto& i = m.info();
    const auto& o = i.origin().position();
    return fmt("%s | %ux%u cells at %.3f m, origin %.2f %.2f", header_str(m.header()).c_str(),
               i.width(), i.height(), i.resolution(), o.x(), o.y());
}

std::string describe(const unitree_go::msg::dds_::SportModeState_& m) {
    const auto& p = m.position();
    const auto& v = m.velocity();
    const auto& rpy = m.imu_state().rpy();
    return fmt("mode %u gait %u | position %.3f %.3f %.3f | velocity %.2f %.2f %.2f, yaw rate "
               "%.2f | rpy %.1f %.1f %.1f deg | body height %.2f",
               m.mode(), m.gait_type(), p[0], p[1], p[2], v[0], v[1], v[2], m.yaw_speed(),
               rpy[0] * 57.2958, rpy[1] * 57.2958, rpy[2] * 57.2958, m.body_height());
}

std::string describe(const unitree_hg::msg::dds_::SportModeState_& m) {
    return fmt("fsm_id %u fsm_mode %u task %u (this type carries no position)", m.fsm_id(),
               m.fsm_mode(), m.task_id());
}

// The payload that matters for bandwidth; the rest of a message is a few hundred bytes.
double payload(const sensor_msgs::msg::dds_::PointCloud2_& m) { return m.data().size() + 200; }
double payload(const nav_msgs::msg::dds_::OccupancyGrid_& m) { return m.data().size() + 200; }
template <typename T> double payload(const T&) { return sizeof(T); }

using Poller = std::function<void(Stat&)>;

template <typename T>
Poller make_reader(dds::domain::DomainParticipant& dp, const std::string& name) {
    auto topic = std::make_shared<dds::topic::Topic<T>>(dp, name);
    dds::sub::Subscriber sub(dp);
    dds::sub::qos::DataReaderQos q = sub.default_datareader_qos();
    q << dds::core::policy::Reliability::BestEffort() << dds::core::policy::History::KeepLast(8);
    auto rd = std::make_shared<dds::sub::DataReader<T>>(sub, *topic, q);
    return [topic, rd](Stat& st) {
        auto samples = rd->take();
        for (const auto& s : samples) {
            if (!s.info().valid()) continue;
            const double t = now_s();
            if (st.count == 0) {
                st.first = t;
                st.sample = describe(s.data());
            }
            st.last = t;
            st.count++;
            st.bytes += payload(s.data());
        }
    };
}

using Factory = std::function<Poller(dds::domain::DomainParticipant&, const std::string&)>;

const std::map<std::string, Factory>& known_types() {
    static const std::map<std::string, Factory> m = {
        {"sensor_msgs::msg::dds_::PointCloud2_",
         make_reader<sensor_msgs::msg::dds_::PointCloud2_>},
        {"nav_msgs::msg::dds_::Odometry_", make_reader<nav_msgs::msg::dds_::Odometry_>},
        {"geometry_msgs::msg::dds_::PoseStamped_",
         make_reader<geometry_msgs::msg::dds_::PoseStamped_>},
        {"sensor_msgs::msg::dds_::Imu_", make_reader<sensor_msgs::msg::dds_::Imu_>},
        {"nav_msgs::msg::dds_::OccupancyGrid_", make_reader<nav_msgs::msg::dds_::OccupancyGrid_>},
        {"unitree_go::msg::dds_::SportModeState_",
         make_reader<unitree_go::msg::dds_::SportModeState_>},
        {"unitree_hg::msg::dds_::SportModeState_",
         make_reader<unitree_hg::msg::dds_::SportModeState_>},
    };
    return m;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string iface = argc > 1 ? argv[1] : "eth0";
    const double discover_s = argc > 2 ? atof(argv[2]) : 5.0;
    const double measure_s = argc > 3 ? atof(argv[3]) : 10.0;
    // Binding Cyclone to the interface is not optional (g1_telemetry_reader learnt it): without
    // it nothing arrives. Everything below attaches to this same domain.
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);

    printf("== DDS discovery on %s, %.0f s\n", iface.c_str(), discover_s);
    fflush(stdout);
    const auto seen = discover(discover_s);
    printf("%zu topics with a writer. '*' = looks map-related, 'M' = measured below\n\n",
           seen.size());
    const auto& known = known_types();
    for (const auto& [name, s] : seen) {
        const char* rel = s.reliable && s.best_effort ? "mixed"
                          : s.reliable                ? "RELIABLE"
                          : s.best_effort             ? "best-effort"
                                                      : "?";
        printf("%c%c %-44s %-48s writers %zu %s\n", interesting(name, s.type) ? '*' : ' ',
               known.count(s.type) ? 'M' : ' ', name.c_str(), s.type.c_str(), s.writers.size(),
               rel);
    }

    std::vector<std::pair<Stat, Poller>> readers;
    dds::domain::DomainParticipant dp(0);
    for (const auto& [name, s] : seen) {
        auto it = known.find(s.type);
        if (it == known.end()) continue;
        Stat st;
        st.topic = name;
        st.type = s.type;
        try {
            readers.emplace_back(st, it->second(dp, name));
        } catch (const std::exception& e) {
            printf("cannot read %s: %s\n", name.c_str(), e.what());
        }
    }
    printf("\n== measuring %zu topics for %.0f s (best-effort readers)\n", readers.size(),
           measure_s);
    fflush(stdout);
    const double start = now_s(), end = start + measure_s;
    while (now_s() < end) {
        for (auto& [st, poll] : readers) poll(st);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (const auto& [st, poll] : readers) {
        const double span = st.count > 1 ? st.last - st.first : 0;
        const double hz = span > 0 ? (st.count - 1) / span : 0;
        const double kbs = st.count ? st.bytes / measure_s / 1024 : 0;
        printf("\n%s  [%s]\n  %ld msgs, %.1f Hz, ~%.0f KB/s%s\n", st.topic.c_str(),
               st.type.c_str(), st.count, hz, kbs,
               st.count == 0 ? "  <- NOTHING ARRIVED (no data, or the writer is silent)" : "");
        if (!st.sample.empty()) printf("  first: %s\n", st.sample.c_str());
    }
    printf("\ndone\n");
    return 0;
}
