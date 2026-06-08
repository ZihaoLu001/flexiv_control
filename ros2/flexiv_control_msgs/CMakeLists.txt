cmake_minimum_required(VERSION 3.8)
project(flexiv_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# action_msgs is mandatory whenever any .action is generated (goal/result/cancel
# infrastructure pulls in action_msgs/msg/GoalInfo, GoalStatus, ...).
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianWaypoint.msg"
  "msg/RobotState.msg"
  "srv/SetMode.srv"
  "action/ExecuteCartesianChunk.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
