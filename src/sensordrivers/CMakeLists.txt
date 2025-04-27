cmake_minimum_required(VERSION 3.8)
project(sensordrivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories("/opt/ros/humble/include/**")
include_directories("/usr/include/pcl-1.12/")
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/opencv4/")

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# link_libraries(/usr/local/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(realsense2 REQUIRED)


add_library(camera_driver SHARED src/CameraDriver.cpp)

ament_target_dependencies(camera_driver 
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  interfaces
  tf2
  tf2_ros
)

target_link_libraries(camera_driver
  MvCameraControl
  yaml-cpp::yaml-cpp
)

rclcpp_components_register_node(
  camera_driver
  PLUGIN "Engineering_robot_RM2025_Pnx::CameraDriver"
  EXECUTABLE camera_driver_node
)

install(TARGETS camera_driver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_executable(mid360_driver src/Livoxmid360Driver.cpp)

ament_target_dependencies(mid360_driver
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
)

target_link_libraries(mid360_driver 
  livox_lidar_sdk_static
  yaml-cpp::yaml-cpp
  )

add_library(realsense_driver SHARED src/RealSenseDriver.cpp)

ament_target_dependencies(realsense_driver 
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  tf2
  tf2_ros
  pcl_conversions
)

target_link_libraries(realsense_driver
  yaml-cpp::yaml-cpp
  realsense2
)

rclcpp_components_register_node(
  realsense_driver
  PLUGIN "Engineering_robot_RM2025_Pnx::RealSense"
  EXECUTABLE realsense_driver_node
)

install(TARGETS realsense_driver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS 
  mid360_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
