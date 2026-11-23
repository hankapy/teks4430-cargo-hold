"""hold_driver: webots_ros2_driver plugin for the cargo_hold robot.

Devices (cargo_hold.wbt):
  fan_motor, fan_sensor        intake fan, velocity control
  damper_motor, damper_sensor  exhaust trunk lid, 0 = closed
  rover_motor, rover_sensor    sampling rover on the deck rail, velocity control
  probe_motor, probe_sensor    gas probe lowered into sampling ports
  status_led                   1 green, 2 yellow, 3 red
  alert_led                    1 red (crew action required)

Subscribes:
  /hold/fan_cmd       (Float64) fan speed, rad/s
  /hold/damper_cmd    (Float64) damper angle, rad
  /hold/rover_vel_cmd (Float64) rover velocity, m/s
  /hold/probe_cmd     (Float64) probe depth, m
  /hold/status_led    (Int32)   0..3
  /hold/alert_led     (Int32)   0..1
Publishes:
  /hold/actuator_state (ActuatorState) measured values + simulation time
"""

import sys
import rclpy
from std_msgs.msg import Float64, Int32
from hold_interfaces.msg import ActuatorState

from hold_autonomy import config as cfg


def _log(msg):
    sys.stderr.write(f"[HoldDriver] {msg}\n")
    sys.stderr.flush()


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class HoldDriver:
    def init(self, webots_node, properties):
        self._ready = False
        self._robot = robot = webots_node.robot
        self._dt = robot.getBasicTimeStep() / 1000.0

        names = ["fan_motor", "fan_sensor", "damper_motor", "damper_sensor", "rover_motor",
                 "rover_sensor", "probe_motor", "probe_sensor", "status_led", "alert_led"]
        d = {}
        for n in names:
            d[n] = robot.getDevice(n)
            if d[n] is None:
                _log(f"ERROR: device '{n}' not found")
        if any(v is None for v in d.values()):
            return
        self._d = d

        ts = int(robot.getBasicTimeStep())
        for n in ("fan_sensor", "damper_sensor", "rover_sensor", "probe_sensor"):
            d[n].enable(ts)

        # velocity control: infinite target position
        for n in ("fan_motor", "rover_motor"):
            d[n].setPosition(float("inf"))
            d[n].setVelocity(0.0)
        d["damper_motor"].setPosition(0.0)
        d["probe_motor"].setPosition(0.0)
        self._prev = {"fan": None, "rover": None}

        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass
        self._node = rclpy.create_node("hold_driver")
        sub = self._node.create_subscription
        sub(Float64, "/hold/fan_cmd", self._on_fan, 10)
        sub(Float64, "/hold/damper_cmd", self._on_damper, 10)
        sub(Float64, "/hold/rover_vel_cmd", self._on_rover, 10)
        sub(Float64, "/hold/probe_cmd", self._on_probe, 10)
        sub(Int32, "/hold/status_led", self._on_status, 10)
        sub(Int32, "/hold/alert_led", self._on_alert, 10)
        self._pub = self._node.create_publisher(ActuatorState, "/hold/actuator_state", 10)

        d["status_led"].set(1)
        self._ready = True
        _log("driver ready")

    # ------------------------------------------------------------ commands

    def _on_fan(self, msg):
        self._d["fan_motor"].setVelocity(_clamp(msg.data, 0.0, cfg.FAN_MAX))

    def _on_damper(self, msg):
        self._d["damper_motor"].setPosition(_clamp(msg.data, 0.0, cfg.DAMPER_MAX))

    def _on_rover(self, msg):
        v = _clamp(msg.data, -cfg.ROVER_VMAX, cfg.ROVER_VMAX)
        pos = self._d["rover_sensor"].getValue()
        # end stops: do not drive past the rail ends
        if (pos <= 0.0 and v < 0.0) or (pos >= cfg.RAIL_LENGTH and v > 0.0):
            v = 0.0
        self._d["rover_motor"].setVelocity(abs(v))
        self._d["rover_motor"].setPosition(float("inf") if v >= 0 else float("-inf"))

    def _on_probe(self, msg):
        self._d["probe_motor"].setPosition(_clamp(msg.data, 0.0, cfg.PROBE_DOWN))

    def _on_status(self, msg):
        self._d["status_led"].set(_clamp(msg.data, 0, 3))

    def _on_alert(self, msg):
        self._d["alert_led"].set(1 if msg.data else 0)

    # ------------------------------------------------------------ step

    def _speed(self, key, pos):
        prev = self._prev[key]
        self._prev[key] = pos
        return 0.0 if prev is None else (pos - prev) / self._dt

    def step(self):
        if not self._ready:
            return
        rclpy.spin_once(self._node, timeout_sec=0)
        d = self._d
        fan_pos = d["fan_sensor"].getValue()
        rover_pos = d["rover_sensor"].getValue()

        m = ActuatorState()
        m.sim_time_s = self._robot.getTime()
        m.fan_speed_rad_s = self._speed("fan", fan_pos)
        m.damper_pos_rad = d["damper_sensor"].getValue()
        fan_f = _clamp(m.fan_speed_rad_s / cfg.FAN_NOMINAL, 0.0, 1.0)
        damper_f = _clamp(m.damper_pos_rad / cfg.DAMPER_OPEN, 0.0, 1.0)
        m.vent_fraction = fan_f * damper_f  # airflow needs both
        m.rover_pos_m = rover_pos
        m.rover_vel_m_s = self._speed("rover", rover_pos)
        m.probe_depth_m = d["probe_sensor"].getValue()
        self._pub.publish(m)
