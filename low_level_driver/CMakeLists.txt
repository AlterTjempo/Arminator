cmake_minimum_required(VERSION 3.8)
project(robot_arm_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# add the robot arm library, consisting of the serial driver and the robot arm driver
add_library(robot_arm_driver SHARED
  src/SerialDriver.cpp
  src/RobotArmDriver.cpp
)

# add the include directories for the library, for both build and install
target_include_directories(robot_arm_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# add boost and threads libraries to the robot arm driver
target_link_libraries(robot_arm_driver PUBLIC Boost::boost)

# installation rules
install(TARGETS robot_arm_driver
    EXPORT robot_arm_driver_Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the public header files to the appropriate include directory
install(DIRECTORY include/
    DESTINATION include
)

# Export the targets for use by other ament packages
ament_export_targets(robot_arm_driver_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)

# add an example executable that uses the robot arm driver library
add_executable(serialtest
  examples/SerialTest.cpp
)

# add the include directories for the example executable
target_include_directories(serialtest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# link the example executable against the robot arm driver library
target_link_libraries(serialtest robot_arm_driver)

# install the example executable
install(TARGETS serialtest
  DESTINATION lib/${PROJECT_NAME}
)

# ament package macro
ament_package()
