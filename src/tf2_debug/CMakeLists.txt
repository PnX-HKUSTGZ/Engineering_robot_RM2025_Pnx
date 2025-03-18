cmake_minimum_required(VERSION 3.8)
project(tf2_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(interfaces REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(tf2_debug src/main.cpp)

ament_target_dependencies(tf2_debug
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  interfaces
  Eigen3
  tf2_ros
  pcl_conversions
  tf2
  tf2_sensor_msgs
)
target_link_libraries(tf2_debug yaml-cpp::yaml-cpp)


install(TARGETS
  tf2_debug
  DESTINATION lib/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
