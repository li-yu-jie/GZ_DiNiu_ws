cmake_minimum_required(VERSION 3.8)
project(diuniu_nav)

# Silence CMake "unused manually-specified variable" warnings. These variables are
# passed by colcon for packages that consume them (e.g., livox_ros_driver2).
if(DEFINED ROS_EDITION)
  # No-op
endif()
if(DEFINED DISTRO_ROS)
  # No-op
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
