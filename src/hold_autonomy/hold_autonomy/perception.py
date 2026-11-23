"""perception: sensing and perception layer.

Fuses the fixed exhaust sensor, rover zone samples and the measured
actuator state into a world model of the hold (see perception_core).

Subscribes:
  /hold/sensors         (HoldSensors)
  /hold/zone_sample     (ZoneSample)
  /hold/actuator_state  (ActuatorState)
Publishes:
  /hold/state           (HoldState) on every exhaust reading
"""

import rclpy
from rclpy.node import Node

from hold_interfaces.msg import HoldSensors, ZoneSample, ActuatorState, HoldState
from hold_autonomy.perception_core import Perception


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.core = Perception()
        self.selfheat_logged = False
        self.fault_logged = False
        self.create_subscription(HoldSensors, "/hold/sensors", self.on_sensors, 10)
        self.create_subscription(ZoneSample, "/hold/zone_sample", self.on_sample, 10)
        self.create_subscription(ActuatorState, "/hold/actuator_state", self.on_actuator, 10)
        self.pub = self.create_publisher(HoldState, "/hold/state", 10)

    def on_actuator(self, a: ActuatorState):
        self.core.on_actuator(a.vent_fraction)

    def on_sample(self, m: ZoneSample):
        self.core.on_zone_sample(m.t_h, m.zone, m.co_ppm, m.ch4_lel, m.o2_pct, m.surface_temp_c)
        z = self.core.zones[m.zone]
        self.get_logger().info(
            f"zone {m.zone} sample at {m.t_h:.1f} h: CO {m.co_ppm:.1f} ppm, "
            f"CO generation {z.co_gen:.2f} ppm/h, surface {m.surface_temp_c:.1f} C"
        )

    def on_sensors(self, m: HoldSensors):
        self.core.on_exhaust(m.t_h, m.co_ppm, m.ch4_lel, m.o2_pct)
        s = self.core.state()
        if s.selfheat and not self.selfheat_logged:
            self.selfheat_logged = True
            self.get_logger().warn(f"self-heating confirmed in zone {s.hotspot_zone} at {s.t_h:.1f} h")
        if s.sensor_fault and not self.fault_logged:
            self.fault_logged = True
            self.get_logger().warn(f"sensor fault at {s.t_h:.1f} h: {s.fault_text}")

        out = HoldState()
        out.t_h = s.t_h
        out.vent_fraction = s.vent_fraction
        out.h_since_vent_stop = s.h_since_vent_stop
        out.raw_reliable = s.raw_reliable
        out.ch4_lel = s.ch4_lel
        out.ch4_exhaust = s.ch4_exhaust
        out.co_exhaust = s.co_exhaust
        out.co_gen_exhaust = s.co_gen_exhaust
        out.o2_exhaust = s.o2_exhaust
        out.zone_co = [float(x) for x in s.zone_co]
        out.zone_co_gen = [float(x) for x in s.zone_co_gen]
        out.zone_temp = [float(x) for x in s.zone_temp]
        out.zone_age_h = [float(x) for x in s.zone_age_h]
        out.zone_suspect = [int(x) for x in s.zone_suspect]
        out.temp_max_c = s.temp_max_c
        out.selfheat = s.selfheat
        out.hotspot_zone = s.hotspot_zone
        out.exhaust_selfheat = s.exhaust_selfheat
        out.sensor_fault = s.sensor_fault
        out.fault_text = s.fault_text
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
