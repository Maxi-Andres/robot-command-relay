// brainco_modbus.hpp — READING the G1's BrainCo Revo 2 hands over Modbus RTU. Nothing else.
//
// THE HANDS (identified 2026-10-15, after two wrong guesses — Dex3-1, then Inspire): two BrainCo
// Revo 2, wired to a small board on the robot's back that reaches PC2 over USB-C as an FTDI
// FT4232H (/dev/ttyUSB0-3). Both answer Modbus RTU at 460800 baud, left as slave 126 and right
// as 127 — BrainCo's factory defaults, and what Unitree's brainco_hand_service hard-codes too.
//
// THE REGISTERS this file reads (input registers, function 04; BrainCo's Revo 2 Modbus docs,
// brainco-hz.com/docs/revolimb-hand/en/revo2/modbus_foundation.html), one block of 24 from 2000:
//   2000-2005  position   0 = fully open .. 1000 = fully closed (register 937 = 0, the default)
//   2006-2011  speed      signed, -1000..1000, positive = closing
//   2012-2017  current    signed
//   2018-2023  motor state 0 idle, 1 running, 2 stalled, 3 turbo
// Finger order in every block: thumb flex, thumb aux (rotation), index, middle, ring, pinky.
//
// READ-ONLY BY CONSTRUCTION: the only request this file can build is function 04. Writing
// (function 06/10h) is a later step with its own safety envelope, and BrainCo lists registers
// that must never be written casually (903 factory reset, 900 OTA, 1000 id, 1001 baud, 1009
// restart) — none of which can be reached from here.
//
// SHARING THE BUS: every transaction takes an exclusive flock() on the port, so a future writer
// in another process can share it without two requests colliding on the wire.
//
// The framing (CRC, request, response checks) is plain C++ with no I/O, tested in CI by
// tests/brainco_test.cpp. Port is the only part that touches a device.
#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace brainco {

constexpr int FINGERS = 6;
constexpr uint8_t LEFT_ID = 126, RIGHT_ID = 127;
constexpr uint16_t STATUS_ADDR = 2000;
constexpr uint16_t STATUS_COUNT = 24;     // position, speed, current, motor state: 6 each
constexpr uint8_t READ_INPUT = 0x04;

inline uint16_t crc16(const uint8_t* p, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (int b = 0; b < 8; ++b) c = (c & 1) ? static_cast<uint16_t>((c >> 1) ^ 0xA001) : c >> 1;
    }
    return c;
}

// The one request this file builds: read `count` input registers from `addr`.
inline std::vector<uint8_t> read_request(uint8_t id, uint16_t addr, uint16_t count) {
    uint8_t f[8] = {id, READ_INPUT, static_cast<uint8_t>(addr >> 8),
                    static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>(count >> 8),
                    static_cast<uint8_t>(count & 0xFF), 0, 0};
    const uint16_t c = crc16(f, 6);
    f[6] = static_cast<uint8_t>(c & 0xFF);         // Modbus sends the CRC low byte first
    f[7] = static_cast<uint8_t>(c >> 8);
    return std::vector<uint8_t>(f, f + 8);
}

inline size_t response_size(uint16_t count) { return 3 + 2 * count + 2; }

// Checks a response to read_request(id, _, count) and fills `regs`. False on anything off: wrong
// size, slave, function (a Modbus exception sets bit 7), byte count or CRC.
inline bool parse_response(const uint8_t* p, size_t n, uint8_t id, uint16_t count,
                           uint16_t* regs) {
    if (n != response_size(count)) return false;
    if (p[0] != id || p[1] != READ_INPUT || p[2] != 2 * count) return false;
    const uint16_t c = crc16(p, n - 2);
    if (p[n - 2] != (c & 0xFF) || p[n - 1] != (c >> 8)) return false;
    for (uint16_t i = 0; i < count; ++i)
        regs[i] = static_cast<uint16_t>((p[3 + 2 * i] << 8) | p[4 + 2 * i]);
    return true;
}

struct Status {
    int pos[FINGERS], speed[FINGERS], current[FINGERS], state[FINGERS];
};

inline Status decode_status(const uint16_t* r) {
    Status s{};
    for (int i = 0; i < FINGERS; ++i) {
        s.pos[i] = r[i];
        s.speed[i] = static_cast<int16_t>(r[6 + i]);
        s.current[i] = static_cast<int16_t>(r[12 + i]);
        s.state[i] = r[18 + i];
    }
    return s;
}

// A serial port at 460800, 8N1, raw. Owns its fd.
class Port {
public:
    explicit Port(const std::string& path) : path_(path) {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return;
        termios t{};
        if (tcgetattr(fd_, &t) != 0) { close(); return; }
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
        cfsetispeed(&t, B460800);
        cfsetospeed(&t, B460800);
        if (tcsetattr(fd_, TCSANOW, &t) != 0) close();
    }
    ~Port() { close(); }
    Port(const Port&) = delete;
    Port& operator=(const Port&) = delete;

    bool ok() const { return fd_ >= 0; }
    const std::string& path() const { return path_; }

    // Reads `count` input registers from slave `id`. False on timeout or a bad answer.
    bool read(uint8_t id, uint16_t addr, uint16_t count, uint16_t* regs, int timeout_ms = 30) {
        if (fd_ < 0) return false;
        const auto req = read_request(id, addr, count);
        const size_t want = response_size(count);
        std::vector<uint8_t> buf(want);
        size_t got = 0;
        flock(fd_, LOCK_EX);
        tcflush(fd_, TCIFLUSH);
        bool sent = ::write(fd_, req.data(), req.size()) == static_cast<ssize_t>(req.size());
        if (sent) {
            tcdrain(fd_);
            const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
            while (got < want && std::chrono::steady_clock::now() < end) {
                const ssize_t r = ::read(fd_, buf.data() + got, want - got);
                if (r > 0) got += static_cast<size_t>(r);
                else usleep(300);
            }
        }
        flock(fd_, LOCK_UN);
        return sent && parse_response(buf.data(), got, id, count, regs);
    }

private:
    void close() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }
    std::string path_;
    int fd_ = -1;
};

}  // namespace brainco
