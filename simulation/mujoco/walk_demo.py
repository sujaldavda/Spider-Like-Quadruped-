"""
Walking demo: crawl gait (one leg lifted at a time) driven by the firmware IK
(firmware/spider_ik/leg_ik.h, compiled and called from Python),
running with full physics through the 12 servo actuators.

Run (from anywhere):
      python Spider/simulation/mujoco/walk_demo.py
      python Spider/simulation/mujoco/walk_demo.py --headless   (simulate 20 s, print results)
"""
import sys, math, time, pathlib
import numpy as np
import mujoco

HERE = pathlib.Path(__file__).resolve().parent   # Spider/simulation/mujoco
SPIDER = HERE.parents[1]                         # Spider/
m = mujoco.MjModel.from_xml_path(str(SPIDER / 'robot_description' / 'mjcf' / 'scene.xml'))
d = mujoco.MjData(m)

# ---------------- IK: the firmware code itself (firmware/spider_ik/leg_ik.h) ----------------
sys.path.insert(0, str(HERE.parent / 'common'))
import leg_ik

LEGS = ['FL', 'RL', 'RR', 'FR']  # same order as the actuators

def solve(leg, x, y, z):
    """foot target in body frame (mm) -> (coxa, femur, tibia) deg, or None if unreachable / past a limit"""
    q, result = leg_ik.inverse(leg, x, y, z)
    return q if result == 'OK' else None

# ---------------- gait ----------------
STANCE = {'FL': (55, 105), 'RL': (-55, 105), 'RR': (-55, -105), 'FR': (55, -105)}  # foot x, y (mm)
HEIGHT = -60.0      # foot z below body origin (mm)
STRIDE = 30.0       # foot travel relative to the body (mm); ground step = STRIDE / (1 - SWING) = 40 mm
LIFT = 20.0         # foot lift (mm)
PERIOD = 2.4        # s for all four legs
SWING = 0.25        # fraction of the cycle a leg is in the air
SWAY = 0.15         # body shift away from the lifted leg (1 = centroid of the other three).
                    # 0.8 = 61 mm side-to-side zigzag; 0.15 = 12 mm; 0 = 2 mm but 2 deg lean onto the lifted corner
ORDER = {'RL': 0.0, 'FL': 0.25, 'RR': 0.5, 'FR': 0.75}  # lateral-sequence crawl
START = 1.5         # s to settle before walking

def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)

def gait_targets(t):
    """body-frame foot targets (mm) for all legs at time t"""
    if t < START:  # stand up from CAD pose to walking stance
        return {n: (STANCE[n][0], STANCE[n][1], HEIGHT) for n in STANCE}, 0.0
    tc = (t - START) / PERIOD
    # body sway: shift toward the three supporting feet, starting before the lift
    sx = sy = 0.0
    for n, off in ORDER.items():
        ph = (tc - off + 0.125) % 1.0          # 0.125..0.375 ~ this leg's swing window
        w = math.sin(math.pi * min(1.0, max(0.0, (ph - 0.0) / 0.5))) ** 2
        sx += w * STANCE[n][0] / 3 * SWAY
        sy += w * STANCE[n][1] / 3 * SWAY
    ramp = smooth((t - START) / 1.0)
    out = {}
    for n, off in ORDER.items():
        ph = (tc - off) % 1.0
        if ph < SWING:
            u = ph / SWING
            dx = -STRIDE / 2 + STRIDE * smooth(u)
            dz = LIFT * math.sin(math.pi * u)
        else:
            u = (ph - SWING) / (1 - SWING)
            dx = STRIDE / 2 - STRIDE * u
            dz = 0.0
        x = STANCE[n][0] + ramp * (dx + sx)
        y = STANCE[n][1] + ramp * sy
        out[n] = (x, y, HEIGHT + ramp * dz)
    return out, ramp

def check_gait():
    bad = 0
    for t in np.arange(0, START + 2 * PERIOD, 0.005):
        tg, _ = gait_targets(t)
        for leg in LEGS:
            if solve(leg, *tg[leg]) is None:
                bad += 1
    return bad

def set_ctrl(t, last=[None]):
    tg, _ = gait_targets(t)
    for i, leg in enumerate(LEGS):
        q = solve(leg, *tg[leg])
        if q is None:
            continue  # keep previous command (should not happen, path is checked)
        d.ctrl[3 * i:3 * i + 3] = np.radians(q)

# start from CAD pose
mujoco.mj_resetDataKeyframe(m, d, 0)
CTRL_DT = 0.01  # 100 Hz like the firmware
bad = check_gait()
print(f'gait path check: {"OK" if bad == 0 else f"{bad} targets unreachable"}')

def run_step():
    if d.time + 1e-9 >= run_step.next_ctrl:
        blend = smooth(d.time / START)  # ease from CAD pose to stance
        tg0 = d.ctrl.copy()
        set_ctrl(d.time)
        d.ctrl[:] = tg0 + (d.ctrl - tg0) * (1 if d.time > START else blend)
        run_step.next_ctrl += CTRL_DT
    mujoco.mj_step(m, d)
run_step.next_ctrl = 0.0

if '--headless' in sys.argv:
    x0, y0 = d.qpos[0], d.qpos[1]
    min_z, max_tilt = 1.0, 0.0
    while d.time < 20.0:
        run_step()
        if d.time > START:
            min_z = min(min_z, d.qpos[2])
            w = d.qpos[3]
            max_tilt = max(max_tilt, math.degrees(2 * math.acos(min(1.0, math.hypot(w, d.qpos[6])))))
    yaw = math.degrees(2 * math.atan2(d.qpos[6], d.qpos[3]))
    walked = d.qpos[0] - x0
    print(f'walked forward {walked * 1000:.0f} mm in {20 - START:.1f} s ({walked / (20 - START) * 1000:.1f} mm/s), '
          f'sideways drift {(d.qpos[1] - y0) * 1000:.0f} mm, yaw {yaw:.1f} deg')
    print(f'lowest body height {min_z * 1000:.1f} mm, max tilt {max_tilt:.1f} deg, '
          f'{"FELL" if min_z < 0.03 or max_tilt > 30 else "stayed up"}')
    sys.exit()

import mujoco.viewer
with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.azimuth, v.cam.elevation, v.cam.distance = 135, -20, 0.6
    v.cam.trackbodyid = m.body('base_link').id
    v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    start = time.time()
    while v.is_running():
        while d.time < time.time() - start:
            run_step()
        v.sync()
        time.sleep(0.005)
