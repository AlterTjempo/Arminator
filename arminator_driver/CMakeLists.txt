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

# add an executable that uses the robot arm driver
add_executable(arminator_driver 
  src/main.cpp
)

# target_include_directories(arminator_driver PRIVATE
#   ../low_level_driver/include
#   ../low_level_driver/src
# )

target_link_libraries(arminator_driver PRIVATE Boost::boost robot_arm_driver::robot_arm_driver)

# install the executable
install(TARGETS arminator_driver
  DESTINATION lib/${PROJECT_NAME}
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
