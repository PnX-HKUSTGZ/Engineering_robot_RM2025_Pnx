cmake_minimum_required(VERSION 3.8)
project(aviadriver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/opt/ros/humble/include/**")
include_directories("/opt/ros/humble/include/**")
include_directories("/usr/include/**/")
include_directories("/usr/include/pcl-1.12/")
include_directories("/opt/ros/humble/include/pcl_msgs/")
include_directories("/opt/ros/humble/include/pcl_conversions/")
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/opencv4/")
link_directories(${PROJECT_SOURCE_DIR}/lib)
# link_libraries(/usr/local/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(AVIA_Driver src/lds_lidar.cpp)

ament_target_dependencies(AVIA_Driver
  rclcpp
  sensor_msgs
  cv_bridge
  rclcpp_components
  interfaces
  tf2
  tf2_ros
)

target_link_libraries(AVIA_Driver
  yaml-cpp::yaml-cpp
  liblivox_sdk_static.a
)

install(TARGETS 
  AVIA_Driver
  DESTINATION lib/${PROJECT_NAME})


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
