cmake_minimum_required(VERSION 3.8)
project(arminator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# include directories
include_directories(include)

# Core library for AL5D robot arm
add_library(al5d_controller SHARED
  src/al5d_controller.cpp
  src/serial_interface.cpp
  src/kinematics.cpp
)

target_include_directories(al5d_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(al5d_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# ROS node executable
add_executable(arminator_node src/arminator_node.cpp)
target_link_libraries(arminator_node al5d_controller)

ament_target_dependencies(arminator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)

# install targets
install(TARGETS
  al5d_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  arminator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_al5d_controller test/test_al5d_controller.cpp)
  target_link_libraries(test_al5d_controller al5d_controller)
endif()

ament_package()