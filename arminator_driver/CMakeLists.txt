cmake_minimum_required(VERSION 3.8)
project(arminator_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)   # optionally add components
find_package(robot_arm_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)


# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveServo.srv"
  "srv/SetPosition.srv"
  "srv/Estop.srv"
)

# Executable
add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    rclcpp::rclcpp
    robot_arm_driver::robot_arm_driver
    ${cpp_typesupport_target}
)


# Install
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
