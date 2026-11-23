"""Control logic (no ROS).

VentController   PI control of CH4 with fan speed, damper/fan interlock
RoverController  sampling rover: state machine + PID position control
"""

from dataclasses import dataclass
from typing import Optional

from hold_autonomy import config as cfg

CLOSED, CONTROL_CH4, PURGE = 0, 1, 2


class PID:
    """PID with output clamping, conditional integration (anti-windup)
    and derivative on measurement."""

    def __init__(self, kp, ki=0.0, kd=0.0, out_min=-1e9, out_max=1e9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_meas = None

    def reset(self, integral=0.0):
        self.integral = integral
        self.prev_meas = None

    def update(self, setpoint, meas, dt):
        e = setpoint - meas
        d = 0.0
        if self.prev_meas is not None and dt > 0:
            d = -(meas - self.prev_meas) / dt
        self.prev_meas = meas
        u_unsat = self.kp * e + self.ki * self.integral + self.kd * d
        u = min(max(u_unsat, self.out_min), self.out_max)
        # anti-windup: skip integration that pushes further into saturation
        inc = self.ki * e
        if dt > 0 and not ((u_unsat > self.out_max and inc > 0) or (u_unsat < self.out_min and inc < 0)):
            self.integral += e * dt
        return u


# ---------------------------------------------------------------- ventilation

VENT_KP = 0.10  # vent fraction per % LEL
VENT_KI = 0.05  # vent fraction per (% LEL * h)
VENT_KD = 0.0  # vent fraction per (% LEL / h)
VENT_MIN = 0.05  # minimum flow in CONTROL_CH4, keeps damper open
DAMPER_READY = 0.9 * cfg.DAMPER_OPEN
FAN_STOPPED = 1.0  # rad/s


class VentController:
    """Maps vent mode to fan and damper commands.

    CONTROL_CH4: PI keeps CH4 at the setpoint with the least airflow,
    so that as little oxygen as possible reaches the cargo.
    Interlock: fan runs only when the damper is open; damper closes only
    after the fan has stopped.
    """

    def __init__(self):
        # error = setpoint - CH4 is negative when CH4 is high, so use -Kp/-Ki
        self.pi = PID(-VENT_KP, -VENT_KI, -VENT_KD, VENT_MIN, 1.0)
        self.mode = CLOSED
        self.t_prev = None
        self.u = 0.0

    def update(self, t_h, mode, ch4, setpoint, damper_pos, fan_speed):
        dt = 0.0 if self.t_prev is None else max(t_h - self.t_prev, 0.0)
        self.t_prev = t_h
        if mode != self.mode:
            if mode == CONTROL_CH4:
                # bumpless start from the current output
                self.pi.reset(integral=self.u / self.pi.ki if self.pi.ki else 0.0)
            self.mode = mode

        if mode == PURGE:
            u = 1.0
        elif mode == CONTROL_CH4:
            u = self.pi.update(setpoint, ch4, dt)
        else:
            u = 0.0
        self.u = u

        want_open = u > 0.02
        if want_open:
            damper_cmd = cfg.DAMPER_OPEN
            fan_cmd = u * cfg.FAN_NOMINAL if damper_pos >= DAMPER_READY else 0.0
        else:
            fan_cmd = 0.0
            damper_cmd = 0.0 if fan_speed < FAN_STOPPED else cfg.DAMPER_OPEN
        return fan_cmd, damper_cmd


def led_commands(alert_level):
    """status_led: 1 green, 2 yellow, 3 red. alert_led on for crew action."""
    status = {0: 1, 1: 2, 2: 2, 3: 3}.get(alert_level, 3)
    return status, 1 if alert_level >= 3 else 0


# ---------------------------------------------------------------- rover

ROVER_KP = 3.0
ROVER_KI = 0.0
ROVER_KD = 0.3
POS_TOL = 0.01  # m
VEL_TOL = 0.02  # m/s
ARRIVE_DWELL = 0.3  # s
SETTLE_TIME = 1.5  # s, gas reaches the probe sensor
PROBE_TOL = 0.01  # m
TIMEOUT = {"MOVE": 20.0, "LOWER": 5.0, "SETTLE": 5.0, "RAISE": 5.0}
FAULT_WAIT = 3.0


@dataclass
class Sample:
    zone: int
    co: float
    ch4: float
    o2: float
    surface_temp: float


class RoverController:
    """States: SELECT -> MOVE -> LOWER -> SETTLE -> SAMPLE -> RAISE -> SELECT.
    Any state that exceeds its timeout goes to FAULT, which retracts the
    probe, waits and retries. Time is simulation time in seconds."""

    def __init__(self):
        self.pid = PID(ROVER_KP, ROVER_KI, ROVER_KD, -cfg.ROVER_VMAX, cfg.ROVER_VMAX)
        self.state = "SELECT"
        self.t_state = 0.0
        self.t_prev = None
        self.target_zone = 0
        self.rr_next = 0  # round-robin index
        self.last_was_priority = False
        self.arrive_since = None
        self.faults = 0
        self.visits = [0] * cfg.N_ZONES
        self.log = []  # (t, state)

    def _enter(self, state, t):
        self.state, self.t_state = state, t
        self.log.append((t, state))

    def _choose(self, priority_zone):
        if 0 <= priority_zone < cfg.N_ZONES and not self.last_was_priority:
            self.last_was_priority = True
            return priority_zone
        self.last_was_priority = False
        z = self.rr_next
        if z == priority_zone:  # skip, it is visited every other time
            z = (z + 1) % cfg.N_ZONES
        self.rr_next = (z + 1) % cfg.N_ZONES
        return z

    def update(self, t, pos, vel, probe, priority_zone=-1, reading: Optional[Sample] = None):
        """Returns (vel_cmd, probe_cmd, sample or None)."""
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        self.t_prev = t
        elapsed = t - self.t_state
        vel_cmd, probe_cmd, sample = 0.0, 0.0, None

        if self.state in TIMEOUT and elapsed > TIMEOUT[self.state]:
            self.faults += 1
            self._enter("FAULT", t)
            elapsed = 0.0

        if self.state == "SELECT":
            self.target_zone = self._choose(priority_zone)
            self.pid.reset()
            self.arrive_since = None
            self._enter("MOVE", t)

        elif self.state == "MOVE":
            target = cfg.PORT_POS[self.target_zone]
            vel_cmd = self.pid.update(target, pos, dt)
            if abs(target - pos) < POS_TOL and abs(vel) < VEL_TOL:
                self.arrive_since = t if self.arrive_since is None else self.arrive_since
                if t - self.arrive_since >= ARRIVE_DWELL:
                    self._enter("LOWER", t)
            else:
                self.arrive_since = None

        elif self.state == "LOWER":
            probe_cmd = cfg.PROBE_DOWN
            if abs(probe - cfg.PROBE_DOWN) < PROBE_TOL:
                self._enter("SETTLE", t)

        elif self.state == "SETTLE":
            probe_cmd = cfg.PROBE_DOWN
            if elapsed >= SETTLE_TIME:
                self._enter("SAMPLE", t)

        elif self.state == "SAMPLE":
            probe_cmd = cfg.PROBE_DOWN
            if reading is not None:
                sample = Sample(self.target_zone, reading.co, reading.ch4, reading.o2, reading.surface_temp)
                self.visits[self.target_zone] += 1
            self._enter("RAISE", t)

        elif self.state == "RAISE":
            if probe < PROBE_TOL:
                self._enter("SELECT", t)

        elif self.state == "FAULT":
            # retract probe and hold position, then retry
            if elapsed >= FAULT_WAIT and probe < PROBE_TOL:
                self._enter("SELECT", t)

        return vel_cmd, probe_cmd, sample
