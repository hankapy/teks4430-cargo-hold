"""Cargo hold monitoring system.

Prerequisites: Webots open with src/worlds/cargo_hold.wbt, simulation running.

Usage:
    ros2 launch hold_autonomy hold.launch.py scenario:=coal_both
Scenarios: coal_methane, coal_selfheat, coal_both, limestone
Sensor fault: co_drift:=1.0 (exhaust CO drift, ppm/h from 24 h)
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    pkg = get_package_share_directory("hold_autonomy")
    urdf = os.path.join(pkg, "resource", "cargo_hold.urdf")

    scenario = LaunchConfiguration("scenario")
    time_scale = ParameterValue(LaunchConfiguration("time_scale"), value_type=float)
    duration = ParameterValue(LaunchConfiguration("duration_h"), value_type=float)
    drift = ParameterValue(LaunchConfiguration("co_drift"), value_type=float)

    driver = WebotsController(
        robot_name="cargo_hold",
        parameters=[{"robot_description": urdf}],
        output="screen",
    )

    def node(name, params=None):
        return Node(package="hold_autonomy", executable=name, parameters=params or [], output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="coal_both"),
        DeclareLaunchArgument("time_scale", default_value="600.0"),
        DeclareLaunchArgument("duration_h", default_value="120.0"),
        DeclareLaunchArgument("co_drift", default_value="0.0"),
        driver,
        node("plant_sim", [{"scenario": scenario, "time_scale": time_scale,
                            "duration_h": duration, "co_drift_ppm_h": drift}]),
        node("perception"),
        node("hold_manager"),
        node("vent_control"),
        node("rover_control"),
        node("recorder", [{"scenario": scenario}]),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
