cmake_minimum_required(VERSION 3.5)
project(agent_visualizer)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "stdc++fs")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "c++fs")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(agent_helpers REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
  ${OMPL_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(timer_callback 
  src/Visualizer.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(timer_callback
  rclcpp
  visualization_msgs
  geometry_msgs
  agent_helpers
  ompl
  Boost
)
target_link_libraries(timer_callback
  yaml-cpp
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# Install the executable
install(TARGETS
  timer_callback
  DESTINATION lib/${PROJECT_NAME}
)

# Macro for ament package
ament_package()
