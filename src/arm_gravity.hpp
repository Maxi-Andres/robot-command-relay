// arm_gravity.hpp — how much torque each G1 arm joint spends holding the arm up, at a pose.
//
// WHY: rule 8 of joint_ctl_core.hpp (a joint pushing into something gives up) compared the
// measured torque with 30% of the rated effort. Holding the arm's own weight already costs
// that much: raised forward with the elbow bent, the left shoulder pitch measured ~8.5 N·m of
// its 25, and the arm then sags a few degrees below its command, so both conditions held and
// the guard stopped an arm that was only being lifted ("Left shoulder pitch pushed against
// something", 2026-10-16). The guard now looks at the torque LEFT OVER after gravity.
//
// HOW: forward kinematics from the pelvis (oriented by the IMU's roll and pitch; yaw does not
// change gravity) through the waist to each arm, with the joint origins, axes, link masses and
// centres of mass of g1_arm_model.hpp (generated from Unitree's URDF). For each arm joint, the
// torque gravity puts about its axis is the sum over the links it carries of
// (com - joint) x (m g); the motor must supply the opposite, which is what this returns, in
// the same sign as the motor's own tau_est.
//
// Pure C++17, no SDK: tests/joint_ctl_test.cpp checks it against the Python model that was
// compared with the robot's measured torques (tools/gen_g1_arm_model.py has the numbers).
#pragma once

#include <cmath>

#include "g1_arm_model.hpp"

namespace jointctl {
namespace gravity {

struct V3 {
    float x = 0, y = 0, z = 0;
};
struct M3 {
    float m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

inline V3 add(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 scale(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline V3 v3(const float v[3]) { return {v[0], v[1], v[2]}; }

inline V3 mul(const M3& r, V3 v) {
    return {r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
            r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
            r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z};
}
inline M3 mul(const M3& a, const M3& b) {
    M3 c;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            c.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return c;
}

// URDF's fixed-axis roll-pitch-yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll).
inline M3 rpy(float r, float p, float y) {
    const float cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p),
                cy = std::cos(y), sy = std::sin(y);
    M3 m;
    m.m[0][0] = cy * cp; m.m[0][1] = cy * sp * sr - sy * cr; m.m[0][2] = cy * sp * cr + sy * sr;
    m.m[1][0] = sy * cp; m.m[1][1] = sy * sp * sr + cy * cr; m.m[1][2] = sy * sp * cr - cy * sr;
    m.m[2][0] = -sp;     m.m[2][1] = cp * sr;                m.m[2][2] = cp * cr;
    return m;
}

// Rotation by angle t about unit axis a (Rodrigues).
inline M3 about(V3 a, float t) {
    const float c = std::cos(t), s = std::sin(t), k = 1 - c;
    M3 m;
    m.m[0][0] = c + a.x * a.x * k;       m.m[0][1] = a.x * a.y * k - a.z * s; m.m[0][2] = a.x * a.z * k + a.y * s;
    m.m[1][0] = a.y * a.x * k + a.z * s; m.m[1][1] = c + a.y * a.y * k;       m.m[1][2] = a.y * a.z * k - a.x * s;
    m.m[2][0] = a.z * a.x * k - a.y * s; m.m[2][1] = a.z * a.y * k + a.x * s; m.m[2][2] = c + a.z * a.z * k;
    return m;
}

constexpr float G = 9.81f;

}  // namespace gravity

// Body indices (the SDK's 29-DoF order) of the waist and of each arm's first joint.
constexpr int WAIST_BODY = 12;            // waist yaw, roll, pitch: 12, 13, 14
constexpr int ARM_BODY[2] = {15, 22};     // left, right shoulder pitch

// The torque each arm joint's motor spends holding the arm still at body pose `q` (29 joint
// angles, rad), with the pelvis tilted by the IMU's `roll` and `pitch` (rad). Writes body
// indices 15..28 of `tau`; the rest are left alone.
inline void arm_gravity(const float* q, float roll, float pitch, float* tau) {
    using namespace gravity;
    // Pelvis -> torso.
    M3 r = rpy(roll, pitch, 0.0f);
    V3 p;
    for (int k = 0; k < 3; ++k) {
        const ArmLink& j = WAIST_CHAIN[k];
        p = add(p, mul(r, v3(j.xyz)));
        r = mul(mul(r, rpy(j.rpy[0], j.rpy[1], j.rpy[2])), about(v3(j.axis), q[WAIST_BODY + k]));
    }
    const M3 torso_r = r;
    const V3 torso_p = p;
    const V3 weight_dir{0.0f, 0.0f, -G};

    for (int side = 0; side < 2; ++side) {
        V3 joint_p[7], joint_axis[7], com[7];
        float mass[7];
        M3 rr = torso_r;
        V3 pp = torso_p;
        for (int k = 0; k < 7; ++k) {
            const ArmLink& j = ARM_CHAIN[side][k];
            pp = add(pp, mul(rr, v3(j.xyz)));
            rr = mul(rr, rpy(j.rpy[0], j.rpy[1], j.rpy[2]));
            joint_p[k] = pp;
            joint_axis[k] = mul(rr, v3(j.axis));
            rr = mul(rr, about(v3(j.axis), q[ARM_BODY[side] + k]));
            com[k] = add(pp, mul(rr, v3(j.com)));
            mass[k] = j.mass;
        }
        for (int k = 0; k < 7; ++k) {
            float t = 0.0f;
            for (int c = k; c < 7; ++c)
                t += dot(cross(sub(com[c], joint_p[k]), scale(weight_dir, mass[c])), joint_axis[k]);
            tau[ARM_BODY[side] + k] = -t;
        }
    }
}

}  // namespace jointctl
