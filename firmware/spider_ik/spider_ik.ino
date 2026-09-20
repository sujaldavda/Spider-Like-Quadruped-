// =====================================================
// SPIDER-LIKE QUADRUPED - 3-DOF IK (all 4 legs)
// ESP8266 + PCA9685 + 12x SG90
// =====================================================
// Coordinates: body frame, mm. Origin = centre of base plate,
// x forward, y left, z up. Same frame as model/quadruped.urdf.
// Joint angles: deg, 0 = CAD assembly pose.
//
// Serial 115200, commands (one per line):
//   help
//   home                      all feet to CAD pose
//   foot FL x y z             move foot to body-frame target
//   rel FL dx dy dz           move foot relative to its home position
//   height dz                 all feet shifted dz from home (negative = body up)
//   joint FL coxa femur tibia set joint angles directly
//   raw ch deg                raw servo angle 0-180 (calibration)
//   where                     print feet and joint angles
// Legs: FL RL RR FR
// =====================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "leg_ik.h"

using namespace leg_ik;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// =====================================================
// PCA9685 / SG90
// =====================================================
#define SERVO_FREQ 50
#define SERVO_MIN 102   // pulse ticks at 0 deg   (~0.5 ms)
#define SERVO_MAX 512   // pulse ticks at 180 deg (~2.5 ms)

// =====================================================
// CALIBRATION  <-- EDIT FOR YOUR ROBOT
// =====================================================
// Index = leg*3 + joint, legs FL RL RR FR, joints coxa femur tibia.
//
// SERVO_CHANNEL: PCA9685 output for each joint.
// SERVO_CENTER : raw servo angle (0-180) that puts the joint in the CAD pose.
//                Find it with "raw ch deg" until the leg matches the CAD.
// SERVO_DIR    : +1 or -1, so positive joint angles move the way the model expects:
//                coxa  +  = leg swings counter-clockwise seen from above (all legs)
//                femur +  = foot UP on FL and RR, foot DOWN on RL and FR
//                tibia +  = foot tip swings outward on FL and RR, inward on RL and FR
//                Test with e.g. "joint FL 0 10 0" and flip the sign if it moves the wrong way.
const uint8_t SERVO_CHANNEL[12] = {
  0, 1, 2,     // FL
  3, 4, 5,     // RL
  6, 7, 8,     // RR
  9, 10, 11    // FR
};
float SERVO_CENTER[12] = {
  90, 90, 90,
  90, 90, 90,
  90, 90, 90,
  90, 90, 90
};
int8_t SERVO_DIR[12] = {
  1, 1, 1,
  1, 1, 1,
  1, 1, 1,
  1, 1, 1
};

// =====================================================
// MOTION
// =====================================================
const unsigned long MOVE_MS = 400;   // duration of a foot move
const unsigned long STEP_MS = 20;    // update period (one servo frame)

float homePos[4][3];   // foot positions at CAD pose
float curPos[4][3];    // current commanded foot positions
Joints curJoints[4];

// =====================================================
// SERVO OUTPUT
// =====================================================
void writeServoRaw(uint8_t index, float deg)
{
  deg = constrain(deg, 0.0, 180.0);
  int pulse = SERVO_MIN + (int)(deg / 180.0 * (SERVO_MAX - SERVO_MIN) + 0.5);
  pca.setPWM(SERVO_CHANNEL[index], 0, pulse);
}

bool writeLeg(int leg, const Joints &q)
{
  float qa[3] = {q.coxa, q.femur, q.tibia};
  float raw[3];
  for (int j = 0; j < 3; j++) {
    int i = leg * 3 + j;
    raw[j] = SERVO_CENTER[i] + SERVO_DIR[i] * qa[j];
    if (raw[j] < 0 || raw[j] > 180) {
      Serial.printf("ERROR: %s joint %d needs servo %.1f deg (outside 0-180), check SERVO_CENTER\n",
                    LEGS[leg].name, j, raw[j]);
      return false;
    }
  }
  for (int j = 0; j < 3; j++) writeServoRaw(leg * 3 + j, raw[j]);
  curJoints[leg] = q;
  return true;
}

// =====================================================
// IK WRAPPERS
// =====================================================
const char *resultText(Result r)
{
  switch (r) {
    case TOO_CLOSE:    return "target too close to coxa axis";
    case OUT_OF_REACH: return "target out of reach";
    case JOINT_LIMIT:  return "joint limit";
    default:           return "ok";
  }
}

bool solveLeg(int leg, float x, float y, float z, Joints &q)
{
  Result r = inverse(LEGS[leg], x, y, z, q);
  if (r != OK) {
    Serial.printf("ERROR: %s -> (%.1f, %.1f, %.1f): %s", LEGS[leg].name, x, y, z, resultText(r));
    if (r == JOINT_LIMIT) Serial.printf(" (coxa %.1f femur %.1f tibia %.1f)", q.coxa, q.femur, q.tibia);
    Serial.println();
    return false;
  }
  return true;
}

// Move several feet together along straight lines. Checks the whole path first.
bool moveFeet(float target[4][3], bool active[4])
{
  int steps = max(1UL, MOVE_MS / STEP_MS);
  Joints q[4];

  for (int k = 1; k <= steps; k++) {
    float t = (float)k / steps;
    for (int leg = 0; leg < 4; leg++) {
      if (!active[leg]) continue;
      float p[3];
      for (int a = 0; a < 3; a++) p[a] = curPos[leg][a] + (target[leg][a] - curPos[leg][a]) * t;
      if (!solveLeg(leg, p[0], p[1], p[2], q[leg])) return false;
    }
  }

  unsigned long next = millis();
  for (int k = 1; k <= steps; k++) {
    float t = (float)k / steps;
    for (int leg = 0; leg < 4; leg++) {
      if (!active[leg]) continue;
      float p[3];
      for (int a = 0; a < 3; a++) p[a] = curPos[leg][a] + (target[leg][a] - curPos[leg][a]) * t;
      solveLeg(leg, p[0], p[1], p[2], q[leg]);
      if (!writeLeg(leg, q[leg])) return false;
    }
    next += STEP_MS;
    while (millis() < next) delay(1);
  }
  for (int leg = 0; leg < 4; leg++)
    if (active[leg])
      for (int a = 0; a < 3; a++) curPos[leg][a] = target[leg][a];
  return true;
}

bool moveOneFoot(int leg, float x, float y, float z)
{
  float target[4][3];
  bool active[4] = {false, false, false, false};
  memcpy(target, curPos, sizeof(target));
  target[leg][0] = x; target[leg][1] = y; target[leg][2] = z;
  active[leg] = true;
  return moveFeet(target, active);
}

// =====================================================
// SERIAL COMMANDS
// =====================================================
int legIndex(const char *s)
{
  if (!s) return -1;
  for (int i = 0; i < 4; i++)
    if (toupper(s[0]) == LEGS[i].name[0] && toupper(s[1]) == LEGS[i].name[1] && s[2] == 0) return i;
  return -1;
}

bool readFloats(float *out, int n)
{
  for (int i = 0; i < n; i++) {
    char *tok = strtok(NULL, " ,\t");
    if (!tok) return false;
    out[i] = atof(tok);
  }
  return true;
}

void printWhere()
{
  for (int leg = 0; leg < 4; leg++)
    Serial.printf("%s foot (%.1f, %.1f, %.1f) mm  joints coxa %.1f femur %.1f tibia %.1f deg\n",
                  LEGS[leg].name, curPos[leg][0], curPos[leg][1], curPos[leg][2],
                  curJoints[leg].coxa, curJoints[leg].femur, curJoints[leg].tibia);
}

void printHelp()
{
  Serial.println("home | foot FL x y z | rel FL dx dy dz | height dz | joint FL c f t | raw ch deg | where");
  Serial.println("legs: FL RL RR FR   units: mm, deg   frame: x forward, y left, z up");
}

void handleCommand(char *line)
{
  char *cmd = strtok(line, " ,\t");
  if (!cmd) return;
  float v[4];

  if (!strcmp(cmd, "help")) {
    printHelp();
  }
  else if (!strcmp(cmd, "home")) {
    bool all[4] = {true, true, true, true};
    moveFeet(homePos, all);
  }
  else if (!strcmp(cmd, "foot") || !strcmp(cmd, "rel")) {
    int leg = legIndex(strtok(NULL, " ,\t"));
    if (leg < 0 || !readFloats(v, 3)) { Serial.println("usage: foot FL x y z | rel FL dx dy dz"); return; }
    if (!strcmp(cmd, "rel")) for (int a = 0; a < 3; a++) v[a] += homePos[leg][a];
    moveOneFoot(leg, v[0], v[1], v[2]);
  }
  else if (!strcmp(cmd, "height")) {
    if (!readFloats(v, 1)) { Serial.println("usage: height dz"); return; }
    float target[4][3];
    bool all[4] = {true, true, true, true};
    for (int leg = 0; leg < 4; leg++) {
      target[leg][0] = homePos[leg][0];
      target[leg][1] = homePos[leg][1];
      target[leg][2] = homePos[leg][2] + v[0];
    }
    moveFeet(target, all);
  }
  else if (!strcmp(cmd, "joint")) {
    int leg = legIndex(strtok(NULL, " ,\t"));
    if (leg < 0 || !readFloats(v, 3)) { Serial.println("usage: joint FL coxa femur tibia"); return; }
    Joints q = {v[0], v[1], v[2]};
    const LegGeom &g = LEGS[leg];
    for (int j = 0; j < 3; j++)
      if (v[j] < g.minDeg[j] || v[j] > g.maxDeg[j]) { Serial.printf("ERROR: joint %d outside %.0f..%.0f\n", j, g.minDeg[j], g.maxDeg[j]); return; }
    if (writeLeg(leg, q)) forward(g, q, curPos[leg][0], curPos[leg][1], curPos[leg][2]);
  }
  else if (!strcmp(cmd, "raw")) {
    if (!readFloats(v, 2) || v[0] < 0 || v[0] > 15) { Serial.println("usage: raw ch deg"); return; }
    int ch = (int)v[0];
    float deg = constrain(v[1], 0.0, 180.0);
    pca.setPWM(ch, 0, SERVO_MIN + (int)(deg / 180.0 * (SERVO_MAX - SERVO_MIN) + 0.5));
    Serial.printf("channel %d -> %.1f deg\n", ch, deg);
    return;
  }
  else if (!strcmp(cmd, "where")) {
    printWhere();
    return;
  }
  else {
    Serial.println("unknown command, type help");
    return;
  }
  printWhere();
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);
  delay(500);

  Joints zero = {0, 0, 0};
  for (int leg = 0; leg < 4; leg++) {
    forward(LEGS[leg], zero, homePos[leg][0], homePos[leg][1], homePos[leg][2]);
    memcpy(curPos[leg], homePos[leg], sizeof(homePos[leg]));
    writeLeg(leg, zero);
  }

  Serial.println();
  Serial.println("================================");
  Serial.println("   QUADRUPED 3-DOF IK READY");
  Serial.println("================================");
  printHelp();
  printWhere();
}

void loop()
{
  static char buf[96];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      handleCommand(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
