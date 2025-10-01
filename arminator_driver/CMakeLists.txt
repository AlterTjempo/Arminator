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
find_package(std_srvs REQUIRED)


# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveServo.srv"
  "srv/SetPosition.srv"
  "srv/Estop.srv"
)

# Get the typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Executable
add_executable(${PROJECT_NAME}_node src/main.cpp)

# Add dependencies to ensure headers are generated before compilation
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  robot_arm_driver
  std_srvs
)

target_link_libraries(${PROJECT_NAME}_node
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
