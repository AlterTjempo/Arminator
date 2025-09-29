cmake_minimum_required(VERSION 3.8)
project(arminator_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(robot_arm_driver REQUIRED)

# SRV generation

find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(ArmSRV
  "srv/MoveServo.srv"
  "srv/SetPosition.srv"
  "srv/Estop.srv"
)

# add an executable that uses the robot arm driver
add_executable(arminator_driver 
  src/main.cpp
)

# target_include_directories(arminator_driver PRIVATE
#   ../low_level_driver/include
#   ../low_level_driver/src
# )


# install the executable
install(TARGETS arminator_driver
  DESTINATION lib/${PROJECT_NAME}
)

#Link something? 

rosidl_get_typesupport_target(cpp_typesupport_target ArmSRV "rosidl_typesupport_cpp")
# target_link_libraries(arminator_driver ${cpp_typesupport_target})
target_link_libraries(arminator_driver PRIVATE Boost::boost robot_arm_driver::robot_arm_driver ${cpp_typesupport_target})


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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
