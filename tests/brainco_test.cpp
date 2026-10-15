// Checks src/brainco_modbus.hpp's framing — the CRC, the one request it can build, and every way
// a response is refused — with no device. Built and run by tests/test_brainco_modbus.py (plain
// g++, so CI runs it). The golden bytes are real answers from the G1's hands, 2026-10-15.
#include "../src/brainco_command.hpp"

#include <cstdio>
#include <vector>

using namespace brainco;

namespace {
int failures = 0;
void check(bool cond, const char* what) {
    std::printf("%s %s\n", cond ? "ok  " : "FAIL", what);
    if (!cond) ++failures;
}
std::vector<uint8_t> with_crc(std::vector<uint8_t> f) {
    const uint16_t c = crc16(f.data(), f.size());
    f.push_back(c & 0xFF);
    f.push_back(c >> 8);
    return f;
}
}  // namespace

int main() {
    // Real answers read off the robot: right hand (127) and left (126), one register.
    const std::vector<uint8_t> right = {0x7f, 0x03, 0x02, 0x00, 0x00, 0x90, 0x4e};
    const std::vector<uint8_t> left = {0x7e, 0x04, 0x02, 0x00, 0x00, 0xac, 0xfa};
    check(crc16(right.data(), 5) == 0x4e90, "CRC matches the right hand's real answer");
    check(crc16(left.data(), 5) == 0xfaac, "CRC matches the left hand's real answer");
    uint16_t one = 0xFFFF;
    check(parse_response(left.data(), left.size(), LEFT_ID, 1, &one) && one == 0,
          "the left hand's real answer parses");
    check(!parse_response(right.data(), right.size(), RIGHT_ID, 1, &one),
          "a function-03 answer is refused: this file only reads input registers (04)");

    const auto req = read_request(LEFT_ID, STATUS_ADDR, STATUS_COUNT);
    check(req.size() == 8 && req[0] == 126 && req[1] == 0x04 && req[2] == 0x07 &&
              req[3] == 0xD0 && req[4] == 0 && req[5] == 24,
          "status request: slave 126, function 04, address 2000, 24 registers");
    check(crc16(req.data(), 6) == (req[6] | (req[7] << 8)), "request CRC, low byte first");

    // A full status answer: positions 0..500, speeds ±, currents, states.
    std::vector<uint8_t> resp = {RIGHT_ID, 0x04, 48};
    const int vals[24] = {0, 100, 200, 300, 400, 500, -5, 10, 0, 0, 0, 1000,
                          120, -80, 0, 0, 0, 0, 0, 1, 2, 0, 0, 3};
    for (int v : vals) {
        resp.push_back(static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF));
        resp.push_back(static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF));
    }
    resp = with_crc(resp);
    uint16_t regs[24];
    check(parse_response(resp.data(), resp.size(), RIGHT_ID, 24, regs), "a status answer parses");
    const Status s = decode_status(regs);
    check(s.pos[5] == 500 && s.speed[0] == -5 && s.speed[5] == 1000, "positions and signed speeds");
    check(s.current[1] == -80 && s.state[1] == 1 && s.state[2] == 2 && s.state[5] == 3,
          "signed currents and motor states");

    auto bad = resp;
    bad[10] ^= 1;
    check(!parse_response(bad.data(), bad.size(), RIGHT_ID, 24, regs), "a corrupted byte is refused");
    check(!parse_response(resp.data(), resp.size(), LEFT_ID, 24, regs),
          "an answer from the other hand is refused");
    check(!parse_response(resp.data(), resp.size() - 1, RIGHT_ID, 24, regs),
          "a short answer is refused");
    const auto exc = with_crc({RIGHT_ID, 0x84, 0x02});
    check(!parse_response(exc.data(), exc.size(), RIGHT_ID, 24, regs),
          "a Modbus exception is refused");
    // The one write: function 10h, registers 1022-1033, six (position, speed) pairs, clamped.
    const int pos[6] = {0, 1000, 1500, -20, 500, 250};
    const int spd[6] = {300, 300, 0, 2000, 300, 300};
    const auto w = targets_request(LEFT_ID, pos, spd);
    check(w.size() == 9 + 24 && w[0] == 126 && w[1] == 0x10 && w[2] == 0x03 && w[3] == 0xFE &&
              w[4] == 0 && w[5] == 12 && w[6] == 24,
          "write: slave 126, function 10h, address 1022, 12 registers, 24 bytes");
    check(crc16(w.data(), w.size() - 2) == (w[w.size() - 2] | (w[w.size() - 1] << 8)), "write CRC");
    auto reg = [&w](int i) { return (w[7 + 2 * i] << 8) | w[8 + 2 * i]; };
    check(reg(0) == 0 && reg(2) == 1000 && reg(4) == 1000 && reg(6) == 0,
          "positions clamped to 0..1000, paired with speeds");
    check(reg(5) == 1 && reg(7) == 1000 && reg(1) == 300, "speeds clamped to 1..1000");
    const auto ack = with_crc({LEFT_ID, 0x10, 0x03, 0xFE, 0x00, 12});
    check(targets_ack(ack.data(), ack.size(), LEFT_ID), "the hand's echo acknowledges the write");
    check(!targets_ack(ack.data(), ack.size(), RIGHT_ID), "an echo from the other hand does not");
    auto wrong = with_crc({LEFT_ID, 0x10, 0x03, 0xFF, 0x00, 12});
    check(!targets_ack(wrong.data(), wrong.size(), LEFT_ID), "an echo of another address does not");
    std::printf(failures ? "FAILED: %d\n" : "PASS\n", failures);
    return failures ? 1 : 0;
}
