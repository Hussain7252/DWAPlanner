cmake_minimum_required(VERSION 3.8)
project(my_dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_util REQUIRED)

pluginlib_export_plugin_description_file(nav2_core plugin/dwa_plugin.xml)

include_directories(
  ${nav_2d_utils_INCLUDE_DIRS}
)

add_library(dwa_planner SHARED src/dwa_code.cpp)
target_include_directories(dwa_planner PUBLIC include)
ament_target_dependencies(dwa_planner rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs nav2_core pluginlib nav_2d_utils nav2_util)
# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the plugin XML file
install(FILES plugin/dwa_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
