cmake_minimum_required(VERSION 3.5)
project(test_bot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Instala assets (config, descripcion, launch, mapas).
install(
  DIRECTORY config description launch maps
  DESTINATION share/${PROJECT_NAME}
)

# Instala los nodos Python como ejecutables del paquete.
# (RENAME quita la extension .py para invocarlos como 'ros2 run test_bot <nombre>'.)
install(PROGRAMS
  scripts/aruco_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_localizer
)
install(PROGRAMS
  scripts/csi_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME csi_camera_node
)

# esp32_bridge_node: en C++ (el puente real con capbot-jetson-bridge no
# necesita Python; ver src/esp32_bridge_node.cpp).
add_executable(esp32_bridge_node src/esp32_bridge_node.cpp)
ament_target_dependencies(esp32_bridge_node
  rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)
install(TARGETS esp32_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
