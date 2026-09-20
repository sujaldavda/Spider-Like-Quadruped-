"""
Python access to the firmware IK (firmware/spider_ik/leg_ik.h).
Compiles leg_ik_bridge.cpp with g++ on first use (and again whenever the .h changes),
so the simulation runs exactly the code that goes on the ESP8266.
Needs g++ installed.
"""
import ctypes, subprocess, pathlib

HERE = pathlib.Path(__file__).resolve().parent   # Spider/simulation/common
SPIDER = HERE.parents[1]                         # Spider/
HEADER = SPIDER / 'firmware' / 'spider_ik' / 'leg_ik.h'
BRIDGE = HERE / 'leg_ik_bridge.cpp'
LIB = HERE / 'build' / 'libleg_ik.so'

LEG_INDEX = {'FL': 0, 'RL': 1, 'RR': 2, 'FR': 3}
RESULT = {0: 'OK', 1: 'TOO_CLOSE', 2: 'OUT_OF_REACH', 3: 'JOINT_LIMIT'}

if not LIB.exists() or LIB.stat().st_mtime < max(HEADER.stat().st_mtime, BRIDGE.stat().st_mtime):
    LIB.parent.mkdir(exist_ok=True)
    subprocess.run(['g++', '-O2', '-shared', '-fPIC', '-o', str(LIB), str(BRIDGE)], check=True)

_lib = ctypes.CDLL(str(LIB))
_F3 = ctypes.c_float * 3
_lib.leg_ik_inverse.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, _F3]
_lib.leg_ik_inverse.restype = ctypes.c_int
_lib.leg_ik_forward.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, _F3]
_lib.leg_ik_forward.restype = None


def inverse(leg, x, y, z):
    """foot target (body frame, mm) -> ((coxa, femur, tibia) deg, result name)"""
    out = _F3()
    r = _lib.leg_ik_inverse(LEG_INDEX[leg], x, y, z, out)
    return (out[0], out[1], out[2]), RESULT[r]


def forward(leg, coxa, femur, tibia):
    """joint angles (deg) -> foot position (body frame, mm)"""
    out = _F3()
    _lib.leg_ik_forward(LEG_INDEX[leg], coxa, femur, tibia, out)
    return out[0], out[1], out[2]
