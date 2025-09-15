cmake_minimum_required(VERSION 3.5)
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
)

# add boost and threads libraries to the robot arm driver
target_link_libraries(robot_arm_driver PUBLIC Threads::Threads Boost::boost)

# installation rules
include(GNUInstallDirs)

# This install rule specifies where to place the built robot_arm_driver library and executable 
# files during installation, and exports its targets for use by other CMake projects.
install(TARGETS robot_arm_driver
    EXPORT robot_arm_driver_Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the public header files to the appropriate include directory
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export the targets to a CMake configuration file for use by other projects
install(EXPORT robot_arm_driver_Targets
    FILE robot_arm_driver-config.cmake
    NAMESPACE robot_arm_driver::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_arm_driver
)

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
