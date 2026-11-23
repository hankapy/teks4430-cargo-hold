"""vent_control: act and control layer for ventilation and status lights.

PI control of CH4 (CONTROL_CH4), full ventilation (PURGE) or closed,
with fan/damper interlock based on measured actuator state.

Subscribes:
  /hold/state, /hold/vent_command, /hold/alert, /hold/actuator_state
Publishes:
  /hold/fan_cmd, /hold/damper_cmd (Float64), /hold/status_led, /hold/alert_led (Int32)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from hold_interfaces.msg import HoldState, VentCommand, Alert, ActuatorState
from hold_autonomy.control_core import VentController, led_commands


class VentControl(Node):
    def __init__(self):
        super().__init__("vent_control")
        self.ctrl = VentController()
        self.cmd = VentCommand()  # default CLOSED
        self.alert_level = 0
        self.act = None
        self.create_subscription(HoldState, "/hold/state", self.on_state, 10)
        self.create_subscription(VentCommand, "/hold/vent_command", self.on_cmd, 10)
        self.create_subscription(Alert, "/hold/alert", self.on_alert, 10)
        self.create_subscription(ActuatorState, "/hold/actuator_state", self.on_actuator, 10)
        self.pub_fan = self.create_publisher(Float64, "/hold/fan_cmd", 10)
        self.pub_damper = self.create_publisher(Float64, "/hold/damper_cmd", 10)
        self.pub_status = self.create_publisher(Int32, "/hold/status_led", 10)
        self.pub_alert = self.create_publisher(Int32, "/hold/alert_led", 10)

    def on_cmd(self, m: VentCommand):
        self.cmd = m

    def on_alert(self, m: Alert):
        self.alert_level = m.level

    def on_actuator(self, m: ActuatorState):
        self.act = m

    def on_state(self, s: HoldState):
        if self.act is None:
            return
        fan, damper = self.ctrl.update(
            s.t_h, self.cmd.mode, s.ch4_exhaust, self.cmd.ch4_setpoint or 10.0,
            self.act.damper_pos_rad, self.act.fan_speed_rad_s,
        )
        self.pub_fan.publish(Float64(data=float(fan)))
        self.pub_damper.publish(Float64(data=float(damper)))
        status, alert = led_commands(self.alert_level)
        self.pub_status.publish(Int32(data=status))
        self.pub_alert.publish(Int32(data=alert))


def main(args=None):
    rclpy.init(args=args)
    node = VentControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
