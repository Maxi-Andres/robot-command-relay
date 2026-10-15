// brainco_command.hpp — the ONE write g1_joint_sender sends to the BrainCo hands: six finger
// positions with their speeds, registers 1022-1033 (function 10h, "write multiple registers").
//
// Read src/brainco_modbus.hpp first (the hands, the ports, the registers it reads). This file is
// separate on purpose: g1_joint_bridge does not include it, so the read-only viewer cannot move
// a finger. Nothing here can reach the registers BrainCo warns about (903 factory reset, 900 OTA,
// 1000 id, 1001 baud, 1009 restart, 1068 auto-calibration) or the current/PWM modes, which drive
// a finger until it stalls: the address and the count below are constants, not parameters.
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

}  // namespace brainco
