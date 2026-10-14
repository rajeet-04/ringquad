#!/usr/bin/env python3
"""Flight performance vs rotor type, with realistic motors (ringquad_control/plant.py).

Run in the container:  python3 ~/ringquad_ws/scripts/realism_sweep.py
"""
import math
import sys
from collections import deque

import numpy as np
from dataclasses import dataclass

from ringquad_control import mixer
from ringquad_control import params as p
from ringquad_control.flight import FlightController, Gains, yaw_of
from ringquad_control.plant import Plant, Rotor, skew

CTL_DT = 0.004
BATTERY_WH = 4 * 3.7 * 3.0 * 0.8   # 4S 3000 mAh, 80% usable
EFFICIENCY = 0.7                   # motor + ESC, electrical -> shaft


@dataclass
class Sensing:
    """What the controller sees: delayed, noisy state (defaults = perfect)."""
    delay_ms: float = 0.0
    pos_sd: float = 0.0     # m
    vel_sd: float = 0.0     # m/s
    att_sd_deg: float = 0.0
    rate_sd: float = 0.0    # rad/s, gyro noise incl. vibration


REAL = Sensing(delay_ms=8.0, pos_sd=0.01, vel_sd=0.03, att_sd_deg=0.3, rate_sd=0.05)


def measure(plant, sensing, rng):
    s = sensing
    rot = plant.rot
    if s.att_sd_deg:
        rot = rot @ (np.eye(3) + skew(rng.normal(0, math.radians(s.att_sd_deg), 3)))
        u, _, vt = np.linalg.svd(rot)
        rot = u @ vt
    return (plant.pos + rng.normal(0, s.pos_sd, 3) if s.pos_sd else plant.pos.copy(),
            plant.vel + rng.normal(0, s.vel_sd, 3) if s.vel_sd else plant.vel.copy(),
            rot,
            plant.rate + rng.normal(0, s.rate_sd, 3) if s.rate_sd else plant.rate.copy())


def fly(plant, gains, pos_sp, yaw_sp, seconds, disturb=None, sensing=Sensing(), seed=0):
    """Closed loop. Returns per-control-step history of (t, pos, tilt_deg, yaw, power)."""
    ctl = FlightController(gains)
    rng = np.random.default_rng(seed)
    steps = round(CTL_DT / plant.dt)
    delayed = deque(maxlen=round(sensing.delay_ms / 1000 / CTL_DT) + 1)
    hist = []
    cmd = mixer.thrusts_to_commands(np.full(4, plant.mass * p.GRAVITY / 4))
    for i in range(int(seconds / plant.dt)):
        t = i * plant.dt
        if disturb:
            plant.ext_force = disturb(t)
        if i % steps == 0:
            delayed.append(measure(plant, sensing, rng))
            pos, vel, rot, rate = delayed[0]
            thrust, torque = ctl.update(pos, vel, rot, rate, np.asarray(pos_sp, float), yaw_sp, CTL_DT)
            cmd = mixer.thrusts_to_commands(mixer.wrench_to_thrusts(thrust, torque))
            hist.append((t, plant.pos.copy(), plant.tilt_deg(), yaw_of(plant.rot), plant.power()))
        plant.step(cmd)
        if plant.tilt_deg() > 80 or np.linalg.norm(plant.pos) > 20:
            hist.append((t, plant.pos.copy(), plant.tilt_deg(), yaw_of(plant.rot), plant.power()))
            break
    return hist


def settle_time(hist, target, tol=0.05):
    target = np.asarray(target)
    last_out = 0.0
    for t, pos, *_ in hist:
        if np.linalg.norm(pos - target) > tol:
            last_out = t
    return last_out if last_out < hist[-1][0] - 0.5 else math.inf


def evaluate(rotor, gains=None, sensing=Sensing()):
    g = gains or Gains()
    r = {}

    # 1 m sideways step at constant altitude
    plant = Plant(rotor=rotor)
    h = fly(plant, g, (1.0, 0, 0), 0.0, 8.0, sensing=sensing)
    xs = np.array([pos[0] for _, pos, *_ in h])
    r['diverged'] = h[-1][2] > 80 or abs(h[-1][1][0] - 1) > 0.5
    r['overshoot_cm'] = max(0.0, (xs.max() - 1.0) * 100)
    r['settle_s'] = settle_time(h, (1, 0, 0))
    r['max_tilt'] = max(tilt for _, _, tilt, *_ in h)

    # 1.5 N sideways gust for 1 s while hovering
    plant = Plant(rotor=rotor)
    h = fly(plant, g, (0, 0, 0), 0.0, 5.0, sensing=sensing,
            disturb=lambda t: np.array([0, 1.5, 0]) if 0.5 < t < 1.5 else np.zeros(3))
    r['gust_cm'] = max(np.linalg.norm(pos) for _, pos, *_ in h) * 100

    # 90 deg yaw
    plant = Plant(rotor=rotor)
    h = fly(plant, g, (0, 0, 0), math.pi / 2, 8.0, sensing=sensing)
    r['yaw_err_deg'] = abs(math.degrees(h[-1][3]) - 90)
    r['yaw_pos_cm'] = max(np.linalg.norm(pos) for _, pos, *_ in h) * 100

    # hover power and endurance
    plant = Plant(rotor=rotor)
    h = fly(plant, g, (0, 0, 0), 0.0, 2.0, sensing=sensing)
    r['hover_w'] = h[-1][4]
    r['minutes'] = BATTERY_WH / (r['hover_w'] / EFFICIENCY) * 60
    return r


def row(name, r):
    if r['diverged']:
        return f"| {name} | **unstable** | | | | | | |"
    settle = f"{r['settle_s']:.1f}" if math.isfinite(r['settle_s']) else 'never'
    return (f"| {name} | {settle} | {r['overshoot_cm']:.0f} | {r['max_tilt']:.0f} | {r['gust_cm']:.0f} | "
            f"{r['yaw_err_deg']:.1f} | {r['hover_w']:.0f} | {r['minutes']:.1f} |")


CONFIGS = {
    'ideal motors, 60 g ring (old Gazebo)': Rotor.ideal(),
    'plain 10-inch prop': Rotor.plain_prop(),
    'ring 20 g': Rotor.ring(0.020),
    'ring 40 g': Rotor.ring(0.040),
    'ring 60 g (current spec)': Rotor.ring(0.060),
}

if __name__ == '__main__':
    gains = Gains()
    only = sys.argv[1:]  # optional name filter
    print('| Rotor | 1 m step settle (s) | overshoot (cm) | max tilt (deg) | 1.5 N gust (cm) | '
          'yaw 90 error (deg) | hover power (W) | flight time (min) |')
    print('|---|---|---|---|---|---|---|---|')
    for name, rotor in CONFIGS.items():
        if only and not any(o in name for o in only):
            continue
        print(row(name, evaluate(rotor, gains)), flush=True)
