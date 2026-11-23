"""Shared geometry and actuator constants. Must match cargo_hold.wbt."""

N_ZONES = 4
HOLD_LENGTH = 11.4  # m, along x
ZONE_CENTERS_X = [-HOLD_LENGTH / 2 + (i + 0.5) * HOLD_LENGTH / N_ZONES for i in range(N_ZONES)]
EXHAUST_ZONE = 3  # exhaust trunk sits above zone 3

# Sampling rail on deck: rover position 0 at x = RAIL_X0
RAIL_X0 = -5.4
RAIL_LENGTH = 10.8
PORT_POS = [x - RAIL_X0 for x in ZONE_CENTERS_X]  # rover coordinates of ports (m)
PORT_TOL = 0.05  # m, probe must be this close to port
HOME_POS = 0.0

# Rover actuators
ROVER_VMAX = 0.8  # m/s
ROVER_ACCEL = 1.0  # m/s^2 (Webots motor acceleration)
PROBE_DOWN = 0.40  # m, probe tip inside port
PROBE_IN_PORT = 0.30  # m, tip below deck
PROBE_SPEED = 0.4  # m/s

# Ventilation actuators
FAN_MAX = 30.0  # rad/s
FAN_NOMINAL = 20.0  # rad/s, full ventilation
FAN_ACCEL = 10.0  # rad/s^2
DAMPER_OPEN = 1.4  # rad
DAMPER_MAX = 1.5
DAMPER_SPEED = 0.7  # rad/s
