cmake_minimum_required(VERSION 3.8)
project(c_strake_conmunication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(serial_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(RMSerialDriver SHARED src/main.cpp src/crc.cpp)

ament_target_dependencies(RMSerialDriver
  rclcpp
  tf2
  serial_driver
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  std_srvs
  Eigen3
  OpenCV
)

target_link_libraries(RMSerialDriver
  yaml-cpp::yaml-cpp
)

rclcpp_components_register_node(
  RMSerialDriver
  PLUGIN "Engineering_robot_RM2025_Pnx::RMSerialDriver"
  EXECUTABLE RMSerialDriver_node
)

install(TARGETS RMSerialDriver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
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
