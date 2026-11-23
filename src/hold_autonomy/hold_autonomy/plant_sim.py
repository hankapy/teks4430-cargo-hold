"""plant_sim: cargo hold process simulation and sensor models.

Process time = Webots simulation time * time_scale. The node is driven by
/hold/actuator_state, which carries the simulation time and the measured
ventilation, rover position and probe depth. The process therefore only
sees what the actuators really do.

Publishes:
  /hold/sensors        (HoldSensors)   fixed exhaust trunk sensor, ~10 Hz
  /hold/probe_reading  (ProbeReading)  air at the rover probe tip
  /hold/truth          (ZoneTruth)     ground truth for evaluation
Subscribes:
  /hold/actuator_state (ActuatorState)
"""

import rclpy
from rclpy.node import Node
import numpy as np

from hold_interfaces.msg import HoldSensors, ProbeReading, ZoneTruth, ActuatorState
from hold_autonomy import process_model as pm
from hold_autonomy.sensor_model import ExhaustSensor, ProbeSensor

MAX_SUBSTEP_H = 1.0 / 60.0  # integration step: 1 min
EXHAUST_PERIOD_S = 0.1  # simulation s


class PlantSim(Node):
    def __init__(self):
        super().__init__("plant_sim")
        self.declare_parameter("scenario", "coal_both")
        self.declare_parameter("time_scale", 600.0)  # process s per simulation s
        self.declare_parameter("duration_h", 120.0)
        self.declare_parameter("seed", 1)
        self.declare_parameter("co_drift_ppm_h", 0.0)  # exhaust CO sensor fault
        self.declare_parameter("co_drift_start_h", 24.0)

        name = self.get_parameter("scenario").value
        if name not in pm.CARGOES:
            raise ValueError(f"unknown scenario '{name}', options: {list(pm.CARGOES)}")
        self.cargo = pm.CARGOES[name]
        self.time_scale = float(self.get_parameter("time_scale").value)
        self.duration_h = float(self.get_parameter("duration_h").value)
        rng = np.random.default_rng(self.get_parameter("seed").value)
        self.exhaust = ExhaustSensor(
            rng,
            drift_ppm_h=float(self.get_parameter("co_drift_ppm_h").value),
            drift_start_h=float(self.get_parameter("co_drift_start_h").value),
        )
        self.probe = ProbeSensor(rng)

        self.hold = pm.HoldState()
        self.t_h = 0.0
        self.sim_t0 = None
        self.last_sim = None
        self.last_exhaust_sim = -1e9
        self.last_log_h = -1
        self.finished = False

        self.pub_sensors = self.create_publisher(HoldSensors, "/hold/sensors", 10)
        self.pub_probe = self.create_publisher(ProbeReading, "/hold/probe_reading", 10)
        self.pub_truth = self.create_publisher(ZoneTruth, "/hold/truth", 10)
        self.create_subscription(ActuatorState, "/hold/actuator_state", self.on_actuator, 10)
        self.get_logger().info(
            f"scenario={name}, time_scale={self.time_scale:.0f} "
            f"(1 simulation s = {self.time_scale / 60:.0f} process min)"
        )

    def on_actuator(self, a: ActuatorState):
        if self.sim_t0 is None:
            self.sim_t0 = self.last_sim = a.sim_time_s
            self.get_logger().info("actuator state received from Webots, process started")
            return
        dt_s = a.sim_time_s - self.last_sim
        self.last_sim = a.sim_time_s
        if dt_s <= 0.0 or self.finished:
            return

        # advance process time in substeps
        dt_total = dt_s * self.time_scale / 3600.0
        n = max(1, int(np.ceil(dt_total / MAX_SUBSTEP_H)))
        for _ in range(n):
            self.hold = pm.step(self.hold, self.cargo, a.vent_fraction, self.t_h, dt_total / n)
            self.t_h += dt_total / n

        # probe sensor follows the rover every step
        r = self.probe.update(self.hold, a.rover_pos_m, a.probe_depth_m, dt_s)
        pr = ProbeReading()
        pr.t_h, pr.co_ppm, pr.ch4_lel, pr.o2_pct, pr.surface_temp_c = self.t_h, r.co, r.ch4, r.o2, r.surface_temp
        self.pub_probe.publish(pr)

        if a.sim_time_s - self.last_exhaust_sim >= EXHAUST_PERIOD_S:
            self.last_exhaust_sim = a.sim_time_s
            e = self.exhaust.read(self.hold, a.vent_fraction, self.t_h)
            hs = HoldSensors()
            hs.t_h, hs.co_ppm, hs.ch4_lel, hs.o2_pct = self.t_h, e.co, e.ch4, e.o2
            self.pub_sensors.publish(hs)
            self.publish_truth()

        if int(self.t_h) != self.last_log_h:
            self.last_log_h = int(self.t_h)
            zs = self.hold.zones
            self.get_logger().info(
                f"t={self.t_h:6.1f} h  T=[{', '.join(f'{z.T:.0f}' for z in zs)}] C  "
                f"CO=[{', '.join(f'{z.CO:.0f}' for z in zs)}] ppm  "
                f"CH4={max(z.CH4 for z in zs):.1f} %LEL  vent={a.vent_fraction:.2f}"
            )
        if self.t_h >= self.duration_h:
            self.finished = True
            self.get_logger().info("scenario finished, shutting down")

    def publish_truth(self):
        zs = self.hold.zones
        m = ZoneTruth()
        m.t_h = self.t_h
        m.temp_c = [float(z.T) for z in zs]
        m.co_ppm = [float(z.CO) for z in zs]
        m.ch4_lel = [float(z.CH4) for z in zs]
        m.o2_pct = [float(z.O2) for z in zs]
        self.pub_truth.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PlantSim()
    try:
        # exit when the scenario ends; the launch file then stops all nodes
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
