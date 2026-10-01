// A command sender whose "robot" only records calls: drives sender_core.hpp for real (stdin
// protocol, clamp, dead-man, EOF stop) with no SDK client and no robot. Built and run by
// tests/test_sender_core.sh. Every call it gets is printed as "call <what>" on stderr.
#include "../src/sender_core.hpp"

#include <cstdio>

int main() {
    return sender::run("fake-sender", {0.3f, 0.2f, 0.5f}, [] {
        sender::Robot r;
        r.move = [](float vx, float vy, float vyaw) {
            fprintf(stderr, "call move %.2f %.2f %.2f\n", vx, vy, vyaw); return 0; };
        r.stop_move = [] { fprintf(stderr, "call stop_move\n"); return 0; };
        r.verbs = {{"stop_move", [] { fprintf(stderr, "call stop_move\n"); return 0; }},
                   {"wave_hand", [] { fprintf(stderr, "call wave_hand\n"); return 0; }},
                   {"pose_on", [] { fprintf(stderr, "call pose_on\n"); return 0; }}};
        r.stops_motion = {"stop_move"};
        r.joy = [](float lx, float ly, float rx, float ry) {
            fprintf(stderr, "call joy %.2f %.2f %.2f %.2f\n", lx, ly, rx, ry); };
        r.joy_on = {"pose_on"};
        return r;
    });
}
