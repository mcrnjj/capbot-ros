cmake_minimum_required(VERSION 3.5)
project(robot_localization)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

set(library_name rl_lib)

# ros_filter.cpp/.hpp #include estos headers generados (srv/set_pose.hpp,
# srv/toggle_filter_processing.hpp, etc.) - sin esto rl_lib no compila.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLL.srv"
  "srv/GetState.srv"
  "srv/SetDatum.srv"
  "srv/SetPose.srv"
  "srv/ToggleFilterProcessing.srv"
  "srv/ToLL.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# navsat_transform.cpp / ros_robot_localization_listener.cpp (y sus nodos)
# se excluyen a propósito: no los usa este proyecto (robot.launch.py solo
# corre ekf_node) y navsat_transform.cpp requiere GeographicLib.
add_library(
  ${library_name}
  src/ekf.cpp
  src/ukf.cpp
  src/filter_base.cpp
  src/filter_utilities.cpp
  src/robot_localization_estimator.cpp
  src/ros_filter.cpp
  src/ros_filter_utilities.cpp
)

rosidl_target_interfaces(${library_name}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(
  ekf_node
  src/ekf_node.cpp
)

add_executable(
  ukf_node
  src/ukf_node.cpp
)

target_link_libraries(
  ${library_name}
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(
  ${library_name}
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(ekf_node ${library_name})
ament_target_dependencies(ekf_node rclcpp)
target_link_libraries(ukf_node ${library_name})
ament_target_dependencies(ukf_node rclcpp)

# Install targets so ros2 run can find the executables
install(TARGETS ekf_node ukf_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
