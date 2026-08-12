cmake_minimum_required(VERSION 3.16)
project(two_towers)

# C++17 is the floor, expressed as a property of the targets rather than as a
# raw -std= flag in add_compile_options. The old form leaked the flag into the
# C sources that rosidl generates for message typesupport, where it is
# meaningless, and it silently did nothing under compilers that were not GCC or
# Clang. BehaviorTree.CPP 4.x also advertises cxx_std_17 on its own interface,
# so anything linking it inherits the same floor either way.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# BehaviorTree.CPP is REQUIRED: the behavior tree node is the only tracker.
# This was once QUIET, which let the build silently succeed without it.
#
# The 4.1 floor is not decorative. The tracker calls Tree::tickOnce(), which
# replaced tickRoot() in 4.1, and the inline XML declares BTCPP_format="4",
# which 3.x cannot parse at all. Against a 3.x install this would otherwise
# fail deep in a template error rather than at configure time.
#
# A floor rather than an exact pin, because the version in the Jazzy apt
# repository moves: it was 4.9.0 when this was written and 4.10.0 is already
# released to the distro. Anything at or above 4.1 has the API this uses.
find_package(behaviortree_cpp 4.1 REQUIRED)

# ============================================================================
# Messages
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TowerStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Must follow rosidl_generate_interfaces. Both C++ executables link this to
# pick up the generated Detection/DetectionArray types.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ============================================================================
# Turret control library
# ============================================================================
# Build without a Raspberry Pi attached (CI, laptops, unit tests).
# See src/cpp/sim/pigpiod_if2.h for what this substitutes.
option(TWO_TOWERS_SIM_GPIO "Stub out pigpio instead of linking the real library" OFF)

add_library(turret_control
  src/cpp/ServoController.cpp
  src/cpp/Turret.cpp
)

target_compile_features(turret_control PUBLIC cxx_std_17)

target_include_directories(turret_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

if(TWO_TOWERS_SIM_GPIO)
  # Put the stub directory ahead of the system includes so that the unmodified
  # `#include <pigpiod_if2.h>` in Turret.cpp resolves to sim/ instead. No source
  # changes and no preprocessor branches in the control code itself.
  target_include_directories(turret_control BEFORE PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/sim>
  )
  message(STATUS "TWO_TOWERS_SIM_GPIO=ON - pigpio stubbed, servos will not move")
else()
  # pigpiod_if2 ships as a bare system library with no CMake config package and
  # no imported target, so it stays a plain name here. PUBLIC because Turret.hpp
  # consumers link through this library.
  target_link_libraries(turret_control PUBLIC pigpiod_if2)
endif()

# ============================================================================
# C++ executable: simple tracker node (no BehaviorTree)
# ============================================================================
add_executable(orthanc_tracker_node src/cpp/orthanc_tracker_node.cpp)

target_link_libraries(orthanc_tracker_node PRIVATE
  turret_control
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  "${cpp_typesupport_target}"
)

# ============================================================================
# C++ executable: BehaviorTree tracker node
# ============================================================================
add_executable(orthanc_tracker_bt_node src/cpp/orthanc_tracker_node_bt.cpp)

target_link_libraries(orthanc_tracker_bt_node PRIVATE
  turret_control
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  behaviortree_cpp::behaviortree_cpp
  "${cpp_typesupport_target}"
)

# ============================================================================
# C++ executable: servo bringup tool (interactive, no ROS2)
# ============================================================================
# Manual pan/tilt jog utility. Used to verify wiring and find the mechanical
# limits of a new turret before running the tracker against it.
#
# SignalGenerator (sine/triangle sweeps) is only used by this tool, so it is
# compiled into the executable rather than into turret_control.
add_executable(servo_bringup
  tests/cpp/menu.cpp
  src/cpp/SignalGenerator.cpp
)

target_link_libraries(servo_bringup PRIVATE turret_control)

# ============================================================================
# Install
# ============================================================================
install(TARGETS turret_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  orthanc_tracker_node
  orthanc_tracker_bt_node
  servo_bringup
  DESTINATION lib/${PROJECT_NAME}
)

# The detector is installed as a plain executable script rather than through
# ament_python_install_package: it is a single file with no importable package
# behind it, and the launch files refer to it by filename.
install(PROGRAMS
  src/python/two_towers_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/cpp/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# These must land in share/, not lib/: FindPackageShare in the launch files
# resolves against the share directory.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# Export
# ============================================================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime behaviortree_cpp rclcpp std_msgs geometry_msgs)

ament_package()
