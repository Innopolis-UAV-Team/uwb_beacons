cmake_minimum_required(VERSION 3.8)
project(uwb_beacons)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
# Note: ament_python_install_package automatically installs entry_points
# from setup.py to the SCRIPTS_DESTINATION
# PACKAGE_DIR should be the directory containing setup.py (the root of the package)
# The first argument is the Python package name, PACKAGE_DIR is where setup.py is
ament_python_install_package(uwb_beacon_ros2
  PACKAGE_DIR uwb_beacon_ros2
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

