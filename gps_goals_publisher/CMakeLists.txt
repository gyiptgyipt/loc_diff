cmake_minimum_required(VERSION 3.8)
project(gps_goals_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies\
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(navigation2 REQUIRED)
find_package(mapviz REQUIRED)
# find_package(tile_map REQUIRED)
find_package(robot_localization REQUIRED)
# find_package(mapviz_plugins REQUIRED)

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


# install(FILES
#         scripts/gps_waypoint_logger.py
#         scripts/interactive_waypoint_follower.py
#         scripts/logged_waypoint_follower.py
#         DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/gps_waypoint_logger.py
        scripts/interactive_waypoint_follower.py
        scripts/logged_waypoint_follower.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
