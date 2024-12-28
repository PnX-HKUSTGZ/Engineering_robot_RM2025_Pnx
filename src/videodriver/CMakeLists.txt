cmake_minimum_required(VERSION 3.8)
project(videodriver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)

# add_library(video_driver SHARED src/VideoDriver.cpp)

# ament_target_dependencies(video_driver
#   rclcpp
#   OpenCV
#   sensor_msgs
#   cv_bridge
#   rclcpp_components
# )

# rclcpp_components_register_nodes(video_driver 
#   PLUGIN "ENGINEER_RM_25::VideoDriver"
#   EXECUTABLE video_driver_node)

# # ament_export_targets(video_driver_node)

# install(TARGETS video_driver
#   EXPORT video_driver_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

add_executable(video_driver src/VideoDriver.cpp)

ament_target_dependencies(video_driver
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
)

install(TARGETS video_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
