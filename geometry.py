"""Fold geometry and sizing for the morphing quad.

Body frame: x forward, y left, z up, origin at the body centre (= hinge height = CoM height).
Computes, for each arm length L and ground arm angle phi:
  - the tilted hinge axis and fold angle (one revolute joint per arm)
  - flight check: ring propellers do not overlap
  - ground checks: rings clear the body and each other
  - ground clearance, wheelbase, track, tip-over threshold, skid-steer ratio

Run: python3 geometry.py            (sweep and print the best candidates)
     python3 geometry.py 0.12 45    (one design: L in metres, phi in degrees)
"""
import math
import sys

# ---- parameters (metres, degrees) ----
RING_D = 0.27        # ring outer diameter (10-inch prop + rim)
RING_W = 0.03        # ring tread width
AXIAL_OFFSET = 0.035 # ring plane offset from arm line along motor axis (above arm in flight, outside body on ground)
BODY_L, BODY_W, BODY_H = 0.16, 0.16, 0.08
GAP = 0.01           # minimum clearance between parts
CAMBER = 10.0        # wheel camber on ground (top of wheel leans inward)
MIN_CLEARANCE = 0.06 # minimum body ground clearance
MAX_SKID_RATIO = 1.4 # wheelbase / track; above this, skid-steer turning gets hard


def dot(a, b): return sum(x * y for x, y in zip(a, b))
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def add(a, b): return [x + y for x, y in zip(a, b)]
def scale(a, k): return [x * k for x in a]
def norm(a):
    n = math.sqrt(dot(a, a))
    return [x / n for x in a]


def rotation_axis_angle(d0, a0, d1, a1):
    """Single rotation taking (arm d0, motor axis a0) to (d1, a1)."""
    f0 = [d0, a0, cross(d0, a0)]
    f1 = [d1, a1, cross(d1, a1)]
    r = [[sum(f1[k][i] * f0[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    ang = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    ax = norm([r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]])
    return ax, math.degrees(ang)


def ring_points(center, axle, n=72):
    """Points on both edges of the ring tread."""
    e1 = norm(cross(axle, [0, 0, 1]) if abs(axle[2]) < 0.99 else cross(axle, [1, 0, 0]))
    e2 = cross(axle, e1)
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        rim = add(center, add(scale(e1, RING_D / 2 * math.cos(t)), scale(e2, RING_D / 2 * math.sin(t))))
        for s in (-RING_W / 2, RING_W / 2):
            pts.append(add(rim, scale(axle, s)))
    return pts


def hits_body(pts):
    return any(abs(p[0]) < BODY_L / 2 + GAP and abs(p[1]) < BODY_W / 2 + GAP and abs(p[2]) < BODY_H / 2 + GAP
               for p in pts)


def evaluate(L, phi_deg):
    """Front-left arm; others are mirror images."""
    hinge = [BODY_L / 2, BODY_W / 2, 0.0]
    phi, c = math.radians(phi_deg), math.radians(CAMBER)

    # flight pose
    d0, a0 = norm([1, 1, 0]), [0, 0, 1]
    fl_air = add(add(hinge, scale(d0, L)), scale(a0, AXIAL_OFFSET))
    side_gap = 2 * fl_air[1] - RING_D          # FL to FR ring edge gap
    fore_gap = 2 * fl_air[0] - RING_D          # FL to RL ring edge gap
    flight_ok = min(side_gap, fore_gap) >= GAP

    # ground pose: axle outward, tilted by camber; arm in wheel plane, angled down by phi
    a1 = [0, math.cos(c), math.sin(c)]
    d1 = [math.cos(phi), 0, -math.sin(phi)]
    d1 = norm(add(d1, scale(a1, -dot(d1, a1))))
    axis, fold = rotation_axis_angle(d0, a0, d1, a1)

    center = add(add(hinge, scale(d1, L)), scale(a1, AXIAL_OFFSET))
    down = norm(add([0, 0, -1], scale(a1, -dot([0, 0, -1], a1))))
    contact = add(center, scale(down, RING_D / 2))
    ground_z = contact[2]

    clearance = (-BODY_H / 2) - ground_z
    wheelbase = 2 * contact[0]
    track = 2 * contact[1]
    com_h = -ground_z                          # CoM at hinge height
    tip_g = (track / 2) / com_h
    same_side_gap = 2 * center[0] - RING_D     # FL to RL ring gap on ground
    body_ok = not hits_body(ring_points(center, a1))

    ok = (flight_ok and body_ok and same_side_gap >= GAP and clearance >= MIN_CLEARANCE
          and wheelbase / track <= MAX_SKID_RATIO)
    return dict(L=L, phi=phi_deg, ok=ok, flight_gap=min(side_gap, fore_gap), body_ok=body_ok,
                same_side_gap=same_side_gap, clearance=clearance, wheelbase=wheelbase, track=track,
                com_h=com_h, tip_g=tip_g, ratio=wheelbase / track, axis=axis, fold=fold)


def show(r):
    print(f"L={r['L']:.3f} m  phi={r['phi']:.0f} deg  ok={r['ok']}")
    print(f"  flight ring gap {r['flight_gap']*100:.1f} cm | ground: body hit={not r['body_ok']}, "
          f"front-rear ring gap {r['same_side_gap']*100:.1f} cm")
    print(f"  clearance {r['clearance']*100:.1f} cm | wheelbase {r['wheelbase']*100:.1f} cm | "
          f"track {r['track']*100:.1f} cm | ratio {r['ratio']:.2f}")
    print(f"  CoM height {r['com_h']*100:.1f} cm | tips over at {r['tip_g']:.2f} g sideways")
    print(f"  hinge axis {[round(x, 3) for x in r['axis']]} | fold {r['fold']:.0f} deg")


if __name__ == "__main__":
    if len(sys.argv) == 3:
        show(evaluate(float(sys.argv[1]), float(sys.argv[2])))
    else:
        results = [evaluate(L / 1000, phi) for L in range(80, 221, 5) for phi in range(0, 81, 5)]
        good = sorted((r for r in results if r["ok"]), key=lambda r: (-round(r["tip_g"], 2), r["L"]))
        print(f"{len(good)} feasible of {len(results)}. Best by tip-over margin, then shortest arm:\n")
        for r in good[:5]:
            show(r)
            print()
