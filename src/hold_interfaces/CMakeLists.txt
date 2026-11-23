cmake_minimum_required(VERSION 3.8)
project(hold_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HoldSensors.msg"
  "msg/ProbeReading.msg"
  "msg/ZoneSample.msg"
  "msg/ZoneTruth.msg"
  "msg/ActuatorState.msg"
  "msg/HoldState.msg"
  "msg/VentCommand.msg"
  "msg/Alert.msg"
  "msg/SamplingPlan.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
