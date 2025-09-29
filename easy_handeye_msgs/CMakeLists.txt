cmake_minimum_required(VERSION 3.8)
project(easy_handeye_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/HandeyeCalibration.msg"
  "msg/SampleList.msg"
  "msg/TargetPoseList.msg"
)

set(srv_files
  "srv/calibration/ListAlgorithms.srv"
  "srv/calibration/SetAlgorithm.srv"
  "srv/calibration/ComputeCalibration.srv"
  "srv/calibration/RemoveSample.srv"
  "srv/calibration/TakeSample.srv"
  "srv/robot_movements/CheckStartingPose.srv"
  "srv/robot_movements/EnumerateTargetPoses.srv"
  "srv/robot_movements/SelectTargetPose.srv"
  "srv/robot_movements/PlanToSelectedTargetPose.srv"
  "srv/robot_movements/ExecutePlan.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()