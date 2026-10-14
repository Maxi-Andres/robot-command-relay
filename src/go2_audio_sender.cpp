// go2_audio_sender — the Go2's own speaker and volume, for the relay. Cannot move the robot.
//
// The Go2's SDK has no audio client (only VUI: volume and LED brightness), but the robot runs
// an "audiohub" service — the one the Unitree app uses — and it answers ordinary SDK requests
// on rt/api/audiohub/request from the Jetson. Its MEGAPHONE mode plays a WAV that is uploaded
// to it in base64 blocks. The api ids are not documented by Unitree; they come from the
// community WebRTC driver (github.com/legion1581/unitree_webrtc_connect, constants.py
// AUDIO_API) and were checked against this robot on 2026-10-14: every call answered 0 and the
// phrase was heard, at 44.1 kHz and at Piper's 22.05 kHz alike.
//
// Protocol, one line in and one line out, like command_sender (the relay serialises it):
//   say <path>      play this WAV. Replies once it is UPLOADED: "ok say <seconds>".
//                   A new `say` cuts the one still playing.
//   stop            cut whatever is playing: "ok stop"
//   volume          "ok volume <0-10>"
//   volume <0-10>   set it: "ok volume <n>"
// Anything else: "err ...". The relay validated the WAV and the level before getting here.
//
// MEASURED 2026-10-14, a 2.6 s phrase at 22.05 kHz: every block costs one ~100 ms round trip
// whatever its size, so 4 KB blocks (the driver's choice) took 3.9 s to upload and 60 KB
// blocks 0.25 s. The service took 60 KB without complaint.
//
// The player's state topic (rt/audiohub/player/state) says "not_in_use" throughout megaphone
// playback, so nothing reports the end of a phrase: the mode is left after the WAV's own
// duration, counted from the end of the upload, plus a margin.

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace unitree::robot;

namespace {

constexpr int API_ENTER_MEGAPHONE = 4001;
constexpr int API_EXIT_MEGAPHONE = 4002;
constexpr int API_UPLOAD_MEGAPHONE = 4003;
constexpr size_t BLOCK = 61440;          // base64 chars per request; a multiple of 4
constexpr double TAIL_S = 0.5;           // after the WAV's duration, before leaving the mode

class AudioHub : public Client {
public:
    AudioHub() : Client("audiohub", false) {}
    void Init() override {
        SetApiVersion("1.0.0.1");
        UT_ROBOT_CLIENT_REG_API_NO_PROI(API_ENTER_MEGAPHONE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(API_EXIT_MEGAPHONE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(API_UPLOAD_MEGAPHONE);
    }
    int32_t Request(int id, const std::string& parameter) {
        std::string data;
        return Call(id, parameter, data);
    }
};

double now_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string base64(const std::vector<unsigned char>& in) {
    static const char* t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((in.size() + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        unsigned v = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
        out += t[v >> 18 & 63]; out += t[v >> 12 & 63]; out += t[v >> 6 & 63]; out += t[v & 63];
    }
    if (i + 1 == in.size()) {
        unsigned v = in[i] << 16;
        out += t[v >> 18 & 63]; out += t[v >> 12 & 63]; out += "==";
    } else if (i + 2 == in.size()) {
        unsigned v = (in[i] << 16) | (in[i + 1] << 8);
        out += t[v >> 18 & 63]; out += t[v >> 12 & 63]; out += t[v >> 6 & 63]; out += '=';
    }
    return out;
}

uint32_t le32(const unsigned char* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

// Seconds of audio in a PCM WAV: the data chunk's size over the fmt chunk's byte rate.
// 0 if either chunk is missing. The relay already refused anything that is not PCM WAV.
double wav_seconds(const std::vector<unsigned char>& w) {
    uint32_t byte_rate = 0, data_size = 0;
    for (size_t i = 12; i + 8 <= w.size();) {
        uint32_t size = le32(&w[i + 4]);
        if (!memcmp(&w[i], "fmt ", 4) && i + 16 <= w.size()) byte_rate = le32(&w[i + 16]);
        if (!memcmp(&w[i], "data", 4)) { data_size = size; break; }
        i += 8 + size + (size & 1);
    }
    return byte_rate ? (double)data_size / byte_rate : 0.0;
}

struct Speaker {
    AudioHub hub;
    bool active = false;
    double leave_at = 0;

    void leave() {
        if (!active) return;
        hub.Request(API_EXIT_MEGAPHONE, "{}");
        active = false;
    }

    std::string say(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        std::vector<unsigned char> wav((std::istreambuf_iterator<char>(f)), {});
        if (wav.size() < 44) return "err say: cannot read " + path;
        const double seconds = wav_seconds(wav);
        if (seconds <= 0) return "err say: no PCM data in " + path;
        leave();                                   // a new phrase cuts the old one
        int32_t rc = hub.Request(API_ENTER_MEGAPHONE, "{}");
        if (rc != 0) return "err say: enter megaphone rc=" + std::to_string(rc);
        active = true;
        const std::string b64 = base64(wav);
        const size_t n = (b64.size() + BLOCK - 1) / BLOCK;
        for (size_t k = 0; k < n; ++k) {
            const std::string part = b64.substr(k * BLOCK, BLOCK);
            const std::string p =
                "{\"current_block_size\":" + std::to_string(part.size()) +
                ",\"block_content\":\"" + part +
                "\",\"current_block_index\":" + std::to_string(k + 1) +
                ",\"total_block_number\":" + std::to_string(n) + "}";
            rc = hub.Request(API_UPLOAD_MEGAPHONE, p);
            if (rc != 0) {
                leave();
                return "err say: upload block " + std::to_string(k + 1) + "/" +
                       std::to_string(n) + " rc=" + std::to_string(rc);
            }
        }
        leave_at = now_s() + seconds + TAIL_S;
        char buf[64];
        snprintf(buf, sizeof buf, "ok say %.2f", seconds);
        return buf;
    }
};

}  // namespace

int main(int argc, char** argv) {
    ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "eth0");
    Speaker speaker;
    speaker.hub.SetTimeout(5.0f);
    speaker.hub.Init();
    go2::VuiClient vui;
    vui.SetTimeout(3.0f);
    vui.Init();

    // Lines are read with read(2) behind poll(2), not std::getline: a buffered stream can hold
    // a line that poll does not see, and the poll timeout is what leaves megaphone mode on time.
    std::string pending;
    char buf[4096];
    for (;;) {
        int timeout_ms = -1;
        if (speaker.active) {
            double left = speaker.leave_at - now_s();
            timeout_ms = left > 0 ? (int)(left * 1000) + 1 : 0;
        }
        pollfd pfd{0, POLLIN, 0};
        int ready = poll(&pfd, 1, timeout_ms);
        if (ready == 0) {                          // the phrase is over
            speaker.leave();
            continue;
        }
        ssize_t got = read(0, buf, sizeof buf);
        if (got <= 0) {                            // the relay is gone: stop talking, then go
            speaker.leave();
            return 0;
        }
        pending.append(buf, got);
        size_t nl;
        while ((nl = pending.find('\n')) != std::string::npos) {
            std::string line = pending.substr(0, nl);
            pending.erase(0, nl + 1);
            std::string reply;
            if (line.rfind("say ", 0) == 0) {
                reply = speaker.say(line.substr(4));
            } else if (line == "stop") {
                speaker.leave();
                reply = "ok stop";
            } else if (line == "volume") {
                int level = -1;
                int32_t rc = vui.GetVolume(level);
                reply = rc == 0 ? "ok volume " + std::to_string(level)
                                : "err volume: get rc=" + std::to_string(rc);
            } else if (line.rfind("volume ", 0) == 0) {
                int level = atoi(line.c_str() + 7);
                int32_t rc = vui.SetVolume(level);
                reply = rc == 0 ? "ok volume " + std::to_string(level)
                                : "err volume: set rc=" + std::to_string(rc);
            } else {
                reply = "err unknown: " + line;
            }
            printf("%s\n", reply.c_str());
            fflush(stdout);
        }
    }
}
