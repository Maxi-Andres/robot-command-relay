// brainco_command.hpp — the TWO writes g1_joint_sender sends to the BrainCo hands (function 10h,
// "write multiple registers"):
//   * six finger positions with their speeds, registers 1022-1033 — every move;
//   * six finger CURRENTS, registers 1040-1045 — only to SQUEEZE an object a grasp has already
//     closed on (rule 12 of joint_ctl_core.hpp, 2026-10-30).
//
// Read src/brainco_modbus.hpp first (the hands, the ports, the registers it reads). This file is
// separate on purpose: g1_joint_bridge does not include it, so the read-only viewer cannot move
// a finger. Nothing here can reach the registers BrainCo warns about (903 factory reset, 900 OTA,
// 1000 id, 1001 baud, 1009 restart, 1068 auto-calibration), nor PWM: every address and count
// below is a constant, not a parameter.
//
// WHY CURRENT MODE NOW. Until 2026-10-30 this file kept the current mode out on purpose (it
// drives a finger until it stalls). Then a grasp on a bottle showed what position mode does when
// a finger meets something: it STOPS there, motor idle, current ~0 — it never squeezes, and the
// bottle was not held. BrainCo's way to grip is current mode: a positive current per finger pushes
// it closed with that much force. Its limits here: gripping direction only (0 .. GRIP_MAX_MA, a
// negative "release" current cannot be sent), under the hand's own protection current (930-935,
// 500 mA by default), and only ever sent by rule 12 after a grasp is confirmed. Any position
// write — open, a slider, the release — puts the finger back in position mode.
//
// Registers 1022-1033 are six (position, speed) pairs in finger order — thumb flex, thumb aux,
// index, middle, ring, pinky. Position 0 open .. 1000 closed; speed 1 .. 1000 (BrainCo's
// "set_finger_positions_and_speeds"). The hand's own protection current (registers 930-935,
// 500 mA by default) still applies and is not touched.
#pragma once

#include "brainco_modbus.hpp"

#include <algorithm>

namespace brainco {

constexpr uint16_t TARGET_ADDR = 1022;
constexpr uint16_t TARGET_COUNT = 12;
constexpr uint8_t WRITE_MULTIPLE = 0x10;

inline std::vector<uint8_t> targets_request(uint8_t id, const int* pos, const int* speed) {
    std::vector<uint8_t> f = {id, WRITE_MULTIPLE, TARGET_ADDR >> 8, TARGET_ADDR & 0xFF, 0,
                              TARGET_COUNT, 2 * TARGET_COUNT};
    for (int i = 0; i < FINGERS; ++i) {
        const int p = std::min(1000, std::max(0, pos[i]));
        const int v = std::min(1000, std::max(1, speed[i]));
        f.push_back(static_cast<uint8_t>(p >> 8));
        f.push_back(static_cast<uint8_t>(p & 0xFF));
        f.push_back(static_cast<uint8_t>(v >> 8));
        f.push_back(static_cast<uint8_t>(v & 0xFF));
    }
    const uint16_t c = crc16(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(c & 0xFF));
    f.push_back(static_cast<uint8_t>(c >> 8));
    return f;
}

// The hand's answer to a 10h write echoes slave, function, address and count.
inline bool targets_ack(const uint8_t* p, size_t n, uint8_t id) {
    if (n != 8 || p[0] != id || p[1] != WRITE_MULTIPLE) return false;
    if (p[2] != (TARGET_ADDR >> 8) || p[3] != (TARGET_ADDR & 0xFF) || p[4] != 0 ||
        p[5] != TARGET_COUNT)
        return false;
    const uint16_t c = crc16(p, 6);
    return p[6] == (c & 0xFF) && p[7] == (c >> 8);
}

// Positions (clamped 0..1000) and speeds (1..1000) for all six fingers. True once acknowledged.
inline bool write_targets(Port& port, uint8_t id, const int* pos, const int* speed) {
    std::vector<uint8_t> resp;
    return port.transact(targets_request(id, pos, speed), 8, resp) &&
           targets_ack(resp.data(), resp.size(), id);
}

constexpr uint16_t CURRENT_ADDR = 1040;
constexpr uint16_t CURRENT_COUNT = 6;
constexpr int GRIP_MAX_MA = 450;      // under the hand's default protection current, 500 mA

// Six gripping currents, mA, clamped to 0 .. GRIP_MAX_MA: never a releasing (negative) one.
inline std::vector<uint8_t> current_request(uint8_t id, const int* ma) {
    std::vector<uint8_t> f = {id, WRITE_MULTIPLE, CURRENT_ADDR >> 8, CURRENT_ADDR & 0xFF, 0,
                              CURRENT_COUNT, 2 * CURRENT_COUNT};
    for (int i = 0; i < FINGERS; ++i) {
        const int c = std::min(GRIP_MAX_MA, std::max(0, ma[i]));
        f.push_back(static_cast<uint8_t>(c >> 8));
        f.push_back(static_cast<uint8_t>(c & 0xFF));
    }
    const uint16_t c = crc16(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(c & 0xFF));
    f.push_back(static_cast<uint8_t>(c >> 8));
    return f;
}

inline bool current_ack(const uint8_t* p, size_t n, uint8_t id) {
    if (n != 8 || p[0] != id || p[1] != WRITE_MULTIPLE) return false;
    if (p[2] != (CURRENT_ADDR >> 8) || p[3] != (CURRENT_ADDR & 0xFF) || p[4] != 0 ||
        p[5] != CURRENT_COUNT)
        return false;
    const uint16_t c = crc16(p, 6);
    return p[6] == (c & 0xFF) && p[7] == (c >> 8);
}

// Squeeze: six gripping currents (mA). True once acknowledged.
inline bool write_current(Port& port, uint8_t id, const int* ma) {
    std::vector<uint8_t> resp;
    return port.transact(current_request(id, ma), 8, resp) &&
           current_ack(resp.data(), resp.size(), id);
}

}  // namespace brainco
