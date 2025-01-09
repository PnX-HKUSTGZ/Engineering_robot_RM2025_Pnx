cmake_minimum_required(VERSION 3.8)
project(videodriver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)
# find_package(MvCameraControl REQUIRED)

add_executable(video_driver src/VideoDriver.cpp)
add_executable(camera_dirver src/CameraDriver.cpp)
add_executable(picture_driver src/PictureDriver.cpp)

ament_target_dependencies(video_driver
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  interfaces
)

target_link_libraries(camera_dirver
  MvCameraControl
)

ament_target_dependencies(camera_dirver
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  interfaces
)

ament_target_dependencies(picture_driver
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  interfaces
)

install(TARGETS video_driver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS camera_dirver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS picture_driver
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
