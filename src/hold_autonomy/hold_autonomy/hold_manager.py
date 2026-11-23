"""hold_manager: reasoning and decision-making layer (py_trees).

Ticks the behaviour tree in decision_core on every world model update.

Subscribes:
  /hold/state          (HoldState)
Publishes:
  /hold/vent_command   (VentCommand)
  /hold/alert          (Alert)
  /hold/sampling_plan  (SamplingPlan)
"""

import rclpy
from rclpy.node import Node

from hold_interfaces.msg import HoldState, VentCommand, Alert, SamplingPlan
from hold_autonomy.perception_core import PerceivedState
from hold_autonomy.decision_core import DecisionMaker

MODE_NAMES = {0: "CLOSED", 1: "CONTROL_CH4", 2: "PURGE"}
LEVEL_NAMES = {0: "NONE", 1: "ADVISORY", 2: "WARNING", 3: "CREW_ACTION"}


def to_perceived(m: HoldState) -> PerceivedState:
    s = PerceivedState()
    for name in ("t_h", "vent_fraction", "h_since_vent_stop", "raw_reliable", "ch4_lel", "ch4_exhaust",
                 "co_exhaust", "co_gen_exhaust", "o2_exhaust", "temp_max_c", "selfheat", "hotspot_zone",
                 "exhaust_selfheat", "sensor_fault", "fault_text"):
        setattr(s, name, getattr(m, name))
    s.zone_co = list(m.zone_co)
    s.zone_co_gen = list(m.zone_co_gen)
    s.zone_temp = list(m.zone_temp)
    s.zone_age_h = list(m.zone_age_h)
    s.zone_suspect = list(m.zone_suspect)
    return s


class HoldManager(Node):
    def __init__(self):
        super().__init__("hold_manager")
        self.dm = DecisionMaker()
        self.last = None
        self.create_subscription(HoldState, "/hold/state", self.on_state, 10)
        self.pub_vent = self.create_publisher(VentCommand, "/hold/vent_command", 10)
        self.pub_alert = self.create_publisher(Alert, "/hold/alert", 10)
        self.pub_plan = self.create_publisher(SamplingPlan, "/hold/sampling_plan", 10)

    def on_state(self, m: HoldState):
        c = self.dm.tick(to_perceived(m))

        v = VentCommand()
        v.mode, v.ch4_setpoint, v.reason = c.vent_mode, c.ch4_setpoint, c.reason
        self.pub_vent.publish(v)
        a = Alert()
        a.level, a.text, a.zone = c.alert_level, c.alert_text, c.alert_zone
        self.pub_alert.publish(a)
        p = SamplingPlan()
        p.priority_zone = c.priority_zone
        self.pub_plan.publish(p)

        # log only when the decision changes
        key = (c.vent_mode, c.alert_level, c.priority_zone, c.ch4_setpoint)
        if key != self.last:
            self.last = key
            self.get_logger().info(
                f"t={m.t_h:.1f} h  mode={MODE_NAMES[c.vent_mode]}  alert={LEVEL_NAMES[c.alert_level]}  "
                f"priority zone={c.priority_zone}  reason: {c.reason}"
                + (f"  | {c.alert_text}" if c.alert_text else "")
                + "\n" + self.dm.ascii()
            )


def main(args=None):
    rclpy.init(args=args)
    node = HoldManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
