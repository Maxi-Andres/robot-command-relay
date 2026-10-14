// g1_audio_sender — the G1's own speaker and volume, for the relay. Cannot move the robot.
//
// Same one-line protocol as go2_audio_sender, so the relay drives both the same way:
//   say <path>      play this WAV. Replies once the first block is queued: "ok say <seconds>".
//                   A new `say` cuts the one still playing.
//   stop            cut whatever is playing: "ok stop"
//   volume          "ok volume <0-100>"
//   volume <0-100>  set it: "ok volume <n>"
//
// Unlike the Go2, the G1's SDK HAS an audio client (g1::AudioClient, the "voice" service), and
// PlayStream is Unitree's documented way to play our own sound (example/g1/audio). Its input is
// raw PCM, 16 kHz, mono, 16-bit — the example refuses anything else — so the WAV is converted
// here: stereo is averaged and the rate is resampled linearly (Piper's voices are 22.05 kHz).
//
// Pacing follows Unitree's example: blocks of 3 s of audio, one per second. Sending a long
// phrase all at once is not something the example does, and nothing documents the service's
// buffer. The blocks after the first are fed from the poll loop, so `say` answers at once and
// a `stop` or a new `say` can cut in between blocks.
//
// NOT YET RUN ON THE ROBOT (written 2026-10-14 with the G1 powered off). The Go2 twin was.

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace unitree::robot;

namespace {

constexpr int RATE = 16000;                  // what PlayStream takes
constexpr size_t BLOCK_BYTES = RATE * 2 * 3; // 3 s of 16-bit mono, as in Unitree's example
constexpr double BLOCK_GAP_S = 1.0;          // ditto
const char* APP = "relay";

double now_s() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

uint32_t le32(const unsigned char* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
uint16_t le16(const unsigned char* p) { return p[0] | p[1] << 8; }

// 16-bit PCM WAV -> 16 kHz mono samples. Empty on anything else (the relay already refused it).
std::vector<int16_t> to_16k_mono(const std::vector<unsigned char>& w) {
    unsigned channels = 0, rate = 0, bits = 0;
    const unsigned char* data = nullptr;
    size_t data_size = 0;
    for (size_t i = 12; i + 8 <= w.size();) {
        uint32_t size = le32(&w[i + 4]);
        if (!memcmp(&w[i], "fmt ", 4) && i + 24 <= w.size()) {
            channels = le16(&w[i + 10]);
            rate = le32(&w[i + 12]);
            bits = le16(&w[i + 22]);
        }
        if (!memcmp(&w[i], "data", 4)) {
            data = &w[i + 8];
            data_size = std::min<size_t>(size, w.size() - i - 8);
            break;
        }
        i += 8 + size + (size & 1);
    }
    std::vector<int16_t> out;
    if (!data || bits != 16 || channels < 1 || channels > 2 || rate == 0) return out;
    const size_t frames = data_size / (2 * channels);
    std::vector<float> mono(frames);
    for (size_t f = 0; f < frames; ++f) {
        int sum = 0;
        for (unsigned c = 0; c < channels; ++c)
            sum += (int16_t)le16(data + 2 * (f * channels + c));
        mono[f] = (float)sum / channels;
    }
    if (rate == RATE) {
        out.assign(mono.begin(), mono.end());
        return out;
    }
    const double step = (double)rate / RATE;
    const size_t n = (size_t)(frames / step);
    out.resize(n);
    for (size_t k = 0; k < n; ++k) {
        const double pos = k * step;
        const size_t a = (size_t)pos;
        const size_t b = std::min(a + 1, frames - 1);
        const double t = pos - a;
        out[k] = (int16_t)(mono[a] * (1 - t) + mono[b] * t);
    }
    return out;
}

struct Speaker {
    g1::AudioClient client;
    std::deque<std::vector<uint8_t>> pending;    // blocks not yet sent
    std::string stream_id;
    double next_at = 0;

    void stop() {
        pending.clear();
        client.PlayStop(APP);
    }

    // Send the next block. False if the service refused it (the rest is dropped).
    bool feed() {
        if (pending.empty()) return true;
        int32_t rc = client.PlayStream(APP, stream_id, pending.front());
        pending.pop_front();
        next_at = now_s() + BLOCK_GAP_S;
        if (rc != 0) {
            fprintf(stderr, "[g1-audio] PlayStream rc=%d, dropping the rest\n", rc);
            pending.clear();
            return false;
        }
        return true;
    }

    std::string say(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        std::vector<unsigned char> wav((std::istreambuf_iterator<char>(f)), {});
        if (wav.size() < 44) return "err say: cannot read " + path;
        const std::vector<int16_t> pcm = to_16k_mono(wav);
        if (pcm.empty()) return "err say: not 16-bit PCM in " + path;
        stop();                                    // a new phrase cuts the old one
        const auto* bytes = reinterpret_cast<const uint8_t*>(pcm.data());
        const size_t total = pcm.size() * 2;
        for (size_t off = 0; off < total; off += BLOCK_BYTES)
            pending.emplace_back(bytes + off, bytes + std::min(total, off + BLOCK_BYTES));
        stream_id = std::to_string((long long)(now_s() * 1000));
        if (!feed()) return "err say: PlayStream refused the first block";
        char buf[64];
        snprintf(buf, sizeof buf, "ok say %.2f", (double)pcm.size() / RATE);
        return buf;
    }
};

}  // namespace

int main(int argc, char** argv) {
    ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "eth0");
    Speaker speaker;
    speaker.client.SetTimeout(5.0f);
    speaker.client.Init();

    // read(2) behind poll(2), as in go2_audio_sender: the poll timeout is what feeds the next
    // block on time, and a buffered stream could hide a line from poll.
    std::string line_buf;
    char buf[4096];
    for (;;) {
        int timeout_ms = -1;
        if (!speaker.pending.empty()) {
            double left = speaker.next_at - now_s();
            timeout_ms = left > 0 ? (int)(left * 1000) + 1 : 0;
        }
        pollfd pfd{0, POLLIN, 0};
        if (poll(&pfd, 1, timeout_ms) == 0) {      // time for the next block
            speaker.feed();
            continue;
        }
        ssize_t got = read(0, buf, sizeof buf);
        if (got <= 0) {                            // the relay is gone: stop talking, then go
            speaker.stop();
            return 0;
        }
        line_buf.append(buf, got);
        size_t nl;
        while ((nl = line_buf.find('\n')) != std::string::npos) {
            std::string line = line_buf.substr(0, nl);
            line_buf.erase(0, nl + 1);
            std::string reply;
            if (line.rfind("say ", 0) == 0) {
                reply = speaker.say(line.substr(4));
            } else if (line == "stop") {
                speaker.stop();
                reply = "ok stop";
            } else if (line == "volume") {
                uint8_t level = 0;
                int32_t rc = speaker.client.GetVolume(level);
                reply = rc == 0 ? "ok volume " + std::to_string(level)
                                : "err volume: get rc=" + std::to_string(rc);
            } else if (line.rfind("volume ", 0) == 0) {
                int level = atoi(line.c_str() + 7);
                int32_t rc = speaker.client.SetVolume((uint8_t)level);
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
