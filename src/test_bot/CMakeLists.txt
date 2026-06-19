cmake_minimum_required(VERSION 3.5)
project(test_bot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

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
install(PROGRAMS
  scripts/esp32_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_bridge_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
