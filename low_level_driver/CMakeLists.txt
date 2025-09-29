cmake_minimum_required(VERSION 3.8)
project(robot_arm_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)

# add an example executable that uses the robot arm driver
add_executable(serialtest
  examples/SerialTest.cpp
  src/SerialDriver.cpp
  src/RobotArmDriver.cpp
)

# add the include directories for the example executable
target_include_directories(serialtest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# link against boost
target_link_libraries(serialtest Boost::boost)

# install the example executable
install(TARGETS serialtest
  DESTINATION lib/${PROJECT_NAME}
)

# install header files for other packages to use
install(DIRECTORY include/
  DESTINATION include
)

install(FILES src/SerialDriver.hpp
  DESTINATION include
)

# export include directories and dependencies
ament_export_include_directories(include)
ament_export_dependencies(Boost)

ament_package()
