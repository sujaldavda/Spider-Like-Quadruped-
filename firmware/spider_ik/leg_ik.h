// =====================================================
// 3-DOF LEG INVERSE KINEMATICS
// =====================================================
// Pure C++ (no Arduino dependencies) so it can be tested on a PC.
//
// Body frame (same as model/quadruped.urdf):
//   origin = centre of base plate, x forward, y left, z up, units mm.
// Joint angles are in degrees with the same zero and sign as the URDF/MJCF:
//   0,0,0 = CAD assembly pose.
//
// Geometry below was measured from the CAD (see model/).
#pragma once
#include <math.h>

namespace leg_ik {

const float DEG = 180.0f / 3.14159265f;

struct LegGeom {
  const char *name;
  float cx, cy, cz;     // coxa axis point (top of coxa servo shaft), body frame
  float psi0;           // leg heading at coxa = 0, deg
  float d;              // sideways offset of leg plane from coxa axis (+ = left of heading)
  float r0, h0;         // femur shaft relative to coxa shaft: outward, up
  float Lf, b1;         // femur shaft -> tibia shaft length, its angle above horizontal at pose 0 (deg)
  float Lt, b2;         // tibia shaft -> foot tip length, its angle above horizontal at pose 0 (deg)
  float s;              // +1: positive femur/tibia angle lifts the foot, -1: lowers it
  float minDeg[3], maxDeg[3];  // joint limits (coxa, femur, tibia) from CAD collision sweep
};

// Order: FL, RL, RR, FR
const LegGeom LEGS[4] = {
  {"FL",  37.520f,  37.166f, 28.55f,   80.468f,  4.6424f, 26.5998f, -7.4f, 41.0598f, 21.2221f, 91.2971f, -79.8526f,  1.0f,
   {-90, -90, -57}, {30, 90, 90}},
  {"RL", -37.166f,  37.520f, 28.55f,  103.603f, -4.3440f, 26.6003f, -6.9f, 41.0593f, 23.3520f, 91.1156f, -77.7618f, -1.0f,
   {-33, -90, -90}, {90, 90, 57}},
  {"RR", -37.520f, -37.166f, 28.55f, -103.603f,  4.7703f, 26.8613f, -6.9f, 41.0593f, 23.3520f, 91.1156f, -77.7618f,  1.0f,
   {-90, -90, -63}, {33, 90, 90}},
  {"FR",  37.166f, -37.520f, 28.55f,  -80.468f, -4.2358f, 26.3102f, -7.4f, 41.0598f, 21.2221f, 91.2971f, -79.8526f, -1.0f,
   {-30, -90, -90}, {90, 90, 57}},
};

struct Joints { float coxa, femur, tibia; };  // deg

enum Result { OK = 0, TOO_CLOSE = 1, OUT_OF_REACH = 2, JOINT_LIMIT = 3 };

inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Foot position (body frame, mm) from joint angles.
inline void forward(const LegGeom &g, const Joints &q, float &x, float &y, float &z) {
  float psi = (g.psi0 + q.coxa) / DEG;
  float a1 = g.b1 / DEG + g.s * q.femur / DEG;                 // femur angle above horizontal
  float a2 = g.b2 / DEG + g.s * (q.femur + q.tibia) / DEG;     // tibia angle above horizontal
  float R = g.r0 + g.Lf * cos(a1) + g.Lt * cos(a2);            // outward distance from coxa axis
  float Z = g.h0 + g.Lf * sin(a1) + g.Lt * sin(a2);
  x = g.cx + R * cos(psi) - g.d * sin(psi);
  y = g.cy + R * sin(psi) + g.d * cos(psi);
  z = g.cz + Z;
}

// Joint angles for a foot target (body frame, mm). Knee-up solution (same as the CAD pose).
// out is always filled when reachable; returns JOINT_LIMIT if any angle is outside the limits.
inline Result inverse(const LegGeom &g, float x, float y, float z, Joints &out) {
  float px = x - g.cx, py = y - g.cy, pz = z - g.cz;

  // 1) Coxa: rotate the leg plane so it contains the foot (plane is offset d from the axis)
  float rho2 = px * px + py * py;
  if (rho2 <= g.d * g.d + 1e-6f) return TOO_CLOSE;
  float R = sqrt(rho2 - g.d * g.d);
  float psi = atan2(py, px) - atan2(g.d, R);
  out.coxa = wrap180(psi * DEG - g.psi0);

  // 2) Femur + tibia: planar 2-link from the femur shaft
  float X = R - g.r0, Z = pz - g.h0;
  float L = sqrt(X * X + Z * Z);
  if (L > g.Lf + g.Lt || L < fabs(g.Lf - g.Lt) || L < 1e-6f) return OUT_OF_REACH;

  float phi = atan2(Z, X);                                                             // direction to foot
  float alpha = acos(clampf((g.Lf * g.Lf + L * L - g.Lt * g.Lt) / (2 * g.Lf * L), -1, 1));  // angle at femur shaft
  float gamma = acos(clampf((g.Lf * g.Lf + g.Lt * g.Lt - L * L) / (2 * g.Lf * g.Lt), -1, 1)); // interior knee angle

  float a1 = phi + alpha;                    // femur above the hip->foot line (knee up)
  float a2 = a1 - (3.14159265f - gamma);     // tibia bends down from the femur

  out.femur = wrap180(g.s * (a1 * DEG - g.b1));
  out.tibia = wrap180(g.s * ((a2 - a1) * DEG - (g.b2 - g.b1)));

  float qa[3] = {out.coxa, out.femur, out.tibia};
  for (int i = 0; i < 3; i++)
    if (qa[i] < g.minDeg[i] || qa[i] > g.maxDeg[i]) return JOINT_LIMIT;
  return OK;
}

}  // namespace leg_ik
