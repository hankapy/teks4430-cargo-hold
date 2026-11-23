"""recorder: writes a CSV log of a run for verification and plots.

One row per log_period_h of process time: ground truth per zone, world
model, decision and actuator state. Zone samples go to a second file.
Plot with: python3 analysis/plot_run.py <csv>
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node

from hold_interfaces.msg import ZoneTruth, HoldState, VentCommand, Alert, ActuatorState, ZoneSample
from hold_autonomy.config import N_ZONES


class Recorder(Node):
    def __init__(self):
        super().__init__("recorder")
        self.declare_parameter("out_dir", "/ros2_ws/logs")
        self.declare_parameter("scenario", "run")
        self.declare_parameter("log_period_h", 0.25)
        out_dir = self.get_parameter("out_dir").value
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(out_dir, f"{self.get_parameter('scenario').value}_{stamp}")
        self.period = float(self.get_parameter("log_period_h").value)

        self.f = open(base + ".csv", "w", newline="")
        self.w = csv.writer(self.f)
        z = range(N_ZONES)
        self.w.writerow(["t_h"] + [f"T{i}" for i in z] + [f"CO{i}" for i in z] + [f"CH4_{i}" for i in z]
                        + ["vent_fraction", "vent_mode", "ch4_setpoint", "alert_level", "ch4_exhaust",
                           "co_exhaust", "selfheat", "hotspot_zone", "sensor_fault", "rover_pos", "sim_time_s"])
        self.fs = open(base + "_samples.csv", "w", newline="")
        self.ws = csv.writer(self.fs)
        self.ws.writerow(["t_h", "zone", "co_ppm", "ch4_lel", "o2_pct", "surface_temp_c"])

        self.truth = self.state = self.act = None
        self.cmd = VentCommand()
        self.alert = Alert()
        self.next_h = 0.0
        self.create_subscription(ZoneTruth, "/hold/truth", self.on_truth, 10)
        self.create_subscription(HoldState, "/hold/state", lambda m: setattr(self, "state", m), 10)
        self.create_subscription(VentCommand, "/hold/vent_command", lambda m: setattr(self, "cmd", m), 10)
        self.create_subscription(Alert, "/hold/alert", lambda m: setattr(self, "alert", m), 10)
        self.create_subscription(ActuatorState, "/hold/actuator_state", lambda m: setattr(self, "act", m), 10)
        self.create_subscription(ZoneSample, "/hold/zone_sample", self.on_sample, 10)
        self.get_logger().info(f"logging to {base}.csv")

    def on_sample(self, m: ZoneSample):
        self.ws.writerow([f"{m.t_h:.3f}", m.zone, f"{m.co_ppm:.2f}", f"{m.ch4_lel:.2f}",
                          f"{m.o2_pct:.2f}", f"{m.surface_temp_c:.2f}"])
        self.fs.flush()

    def on_truth(self, m: ZoneTruth):
        if m.t_h < self.next_h or self.state is None or self.act is None:
            return
        self.next_h = m.t_h + self.period
        s, a = self.state, self.act
        row = ([f"{m.t_h:.3f}"] + [f"{x:.2f}" for x in m.temp_c] + [f"{x:.2f}" for x in m.co_ppm]
               + [f"{x:.2f}" for x in m.ch4_lel]
               + [f"{a.vent_fraction:.3f}", self.cmd.mode, f"{self.cmd.ch4_setpoint:.1f}", self.alert.level,
                  f"{s.ch4_exhaust:.2f}", f"{s.co_exhaust:.2f}", int(s.selfheat), s.hotspot_zone,
                  int(s.sensor_fault), f"{a.rover_pos_m:.3f}", f"{a.sim_time_s:.2f}"])
        self.w.writerow(row)
        self.f.flush()

    def destroy_node(self):
        self.f.close()
        self.fs.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
