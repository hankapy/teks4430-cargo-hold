"""rover_control: act and control layer for the sampling rover.

State machine SELECT -> MOVE -> LOWER -> SETTLE -> SAMPLE -> RAISE with
PID position control on the rail and timeouts to a FAULT state
(see control_core.RoverController). A sample is published only after the
probe has settled in a port, so measurement validity follows from the
actuator state.

Subscribes:
  /hold/actuator_state  (ActuatorState)  time base, position, probe depth
  /hold/probe_reading   (ProbeReading)
  /hold/sampling_plan   (SamplingPlan)
Publishes:
  /hold/rover_vel_cmd   (Float64, m/s)
  /hold/probe_cmd       (Float64, m)
  /hold/zone_sample     (ZoneSample)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from hold_interfaces.msg import ActuatorState, ProbeReading, SamplingPlan, ZoneSample
from hold_autonomy.control_core import RoverController
from hold_autonomy.sensor_model import GasReading


class RoverControl(Node):
    def __init__(self):
        super().__init__("rover_control")
        self.ctrl = RoverController()
        self.reading = None
        self.priority = -1
        self.prev_state = None
        self.create_subscription(ActuatorState, "/hold/actuator_state", self.on_actuator, 10)
        self.create_subscription(ProbeReading, "/hold/probe_reading", self.on_reading, 10)
        self.create_subscription(SamplingPlan, "/hold/sampling_plan", self.on_plan, 10)
        self.pub_vel = self.create_publisher(Float64, "/hold/rover_vel_cmd", 10)
        self.pub_probe = self.create_publisher(Float64, "/hold/probe_cmd", 10)
        self.pub_sample = self.create_publisher(ZoneSample, "/hold/zone_sample", 10)

    def on_reading(self, m: ProbeReading):
        self.reading = m

    def on_plan(self, m: SamplingPlan):
        if m.priority_zone != self.priority:
            self.get_logger().info(f"priority zone set to {m.priority_zone}")
        self.priority = m.priority_zone

    def on_actuator(self, a: ActuatorState):
        r = None
        if self.reading is not None:
            p = self.reading
            r = GasReading(p.co_ppm, p.ch4_lel, p.o2_pct, p.surface_temp_c)
        vel, probe, sample = self.ctrl.update(
            a.sim_time_s, a.rover_pos_m, a.rover_vel_m_s, a.probe_depth_m, self.priority, r
        )
        self.pub_vel.publish(Float64(data=float(vel)))
        self.pub_probe.publish(Float64(data=float(probe)))

        if sample is not None and self.reading is not None:
            m = ZoneSample()
            m.t_h = self.reading.t_h
            m.zone = sample.zone
            m.co_ppm, m.ch4_lel, m.o2_pct, m.surface_temp_c = sample.co, sample.ch4, sample.o2, sample.surface_temp
            self.pub_sample.publish(m)

        if self.ctrl.state != self.prev_state:
            if self.ctrl.state == "FAULT":
                self.get_logger().warn(f"rover fault (timeout), faults so far: {self.ctrl.faults}")
            elif self.ctrl.state == "MOVE":
                self.get_logger().debug(f"moving to zone {self.ctrl.target_zone}")
            self.prev_state = self.ctrl.state


def main(args=None):
    rclpy.init(args=args)
    node = RoverControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
