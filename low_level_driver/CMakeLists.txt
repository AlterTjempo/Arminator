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

# create a library from the robot arm driver sources
add_library(robot_arm_driver
  src/SerialDriver.cpp
  src/RobotArmDriver.cpp
)

# add the include directories for the library
target_include_directories(robot_arm_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# link against boost
target_link_libraries(robot_arm_driver PUBLIC Boost::boost)

# install the library
install(TARGETS robot_arm_driver
  EXPORT robot_arm_driver_Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# install header files
install(DIRECTORY include/
  DESTINATION include
)

# add an example executable that uses the robot arm driver library
add_executable(serialtest
  examples/SerialTest.cpp
)

target_link_libraries(serialtest robot_arm_driver)

# install the example executable
install(TARGETS serialtest
  DESTINATION lib/${PROJECT_NAME}
)

# export the library target for other packages
ament_export_targets(robot_arm_driver_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)

ament_package()
