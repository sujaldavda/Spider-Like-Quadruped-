// C wrapper so Python can call the firmware IK (firmware/spider_ik/leg_ik.h) directly.
// Built automatically by simulation/common/leg_ik.py.
#include "../../firmware/spider_ik/leg_ik.h"

extern "C" {

// leg: 0=FL 1=RL 2=RR 3=FR. Target in body frame, mm. out = coxa, femur, tibia (deg).
// Returns leg_ik::Result (0 = OK).
int leg_ik_inverse(int leg, float x, float y, float z, float *out)
{
  leg_ik::Joints q = {0, 0, 0};
  int r = leg_ik::inverse(leg_ik::LEGS[leg], x, y, z, q);
  out[0] = q.coxa; out[1] = q.femur; out[2] = q.tibia;
  return r;
}

// Joint angles (deg) -> foot position in body frame (mm).
void leg_ik_forward(int leg, float coxa, float femur, float tibia, float *out)
{
  leg_ik::forward(leg_ik::LEGS[leg], {coxa, femur, tibia}, out[0], out[1], out[2]);
}

}
