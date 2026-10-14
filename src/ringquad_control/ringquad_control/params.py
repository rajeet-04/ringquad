"""Vehicle constants. Must match urdf/ringquad.urdf.xacro and DESIGN.md section 3."""
import math

import numpy as np

GRAVITY = 9.81

# Mass: body 1.0 + 4 x (arm with motor 0.11 + ring 0.06)
MASS = 1.0 + 4 * (0.11 + 0.06)

# Rough inertia about the CoM in flight pose (body box + arms and rings as point masses + ring spin inertia).
INERTIA = np.diag([0.021, 0.021, 0.039])

# Rotor centres in the body frame, flight pose: hinge (+-0.08, +-0.08) + 0.12 m along the 45 deg diagonal.
ARM_XY = 0.08 + 0.12 * math.cos(math.radians(45))

# Order = actuator number in the motor model plugin.
ROTOR_NAMES = ('fl', 'fr', 'rl', 'rr')
ROTOR_XY = np.array([
    [ARM_XY, ARM_XY],
    [ARM_XY, -ARM_XY],
    [-ARM_XY, ARM_XY],
    [-ARM_XY, -ARM_XY],
])
# turningDirection: ccw = +1, cw = -1. Joint velocity = direction x command.
ROTOR_DIR = np.array([1.0, -1.0, -1.0, 1.0])

# Ring propeller spin inertia: annulus m/2 (ri^2 + ro^2), m = 0.06, ri = 0.12, ro = 0.135.
# About 12x a plain 10-inch prop. Changing rotor speed kicks the body in yaw like a reaction
# wheel, far more than the aerodynamic drag torque does, so yaw gains must stay low.
RING_IZZ = 0.06 / 2 * (0.12 ** 2 + 0.135 ** 2)
MOTOR_TAU_UP = 0.0125    # s, motor model timeConstantUp
MOTOR_TAU_DOWN = 0.025   # s, motor model timeConstantDown

K_THRUST = 1.56e-5      # N / (rad/s)^2
K_MOMENT = 0.016        # drag torque / thrust (m)
MAX_ROTOR_SPEED = 800.0  # rad/s
MAX_THRUST = K_THRUST * MAX_ROTOR_SPEED ** 2

# ---- Real-hardware limits (used by plant.py and the realistic Gazebo motors) ----
# Motor: 10-inch class, ~700 KV on 4S. Kt = 60 / (2 pi KV) = 0.0136 Nm/A, ~30 A peak.
MOTOR_MAX_TORQUE = 0.4          # Nm
MOTOR_ROTOR_IZZ = 3e-5          # kg m^2, motor bell
# Aerodynamic torque on the blades = K_MOMENT * thrust = PROP_DRAG * w^2.
PROP_DRAG = K_MOMENT * K_THRUST
# Rim skin drag (rough): rim speed ~70 m/s at hover, turbulent flat plate Cf ~ 0.0036 over
# ~0.074 m^2 wetted area at r ~ 0.128 m gives ~0.09 Nm at 514 rad/s.
RIM_DRAG = 0.092 / 514.0 ** 2   # Nm / (rad/s)^2
PLAIN_PROP_IZZ = 8e-5           # kg m^2, plain 10-inch prop (~15 g), for comparison


def ring_izz(mass):
    """Spin inertia of a ring propeller of the given mass (same geometry as the model)."""
    return mass / 2 * (0.12 ** 2 + 0.135 ** 2)

# Ground mode (geometry.py, 12 cm arm, 45 deg down, 10 deg camber)
FOLD_FLIGHT = 0.0
FOLD_GROUND = math.radians(80)
WHEEL_RADIUS = 0.135
TRACK = 0.305
LEFT = np.array([True, False, True, False])  # fl, fr, rl, rr
