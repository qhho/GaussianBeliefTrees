cmake_minimum_required(VERSION 3.5)
project(agent_gaussian_belief_trees)
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

# Include C++ directories
include_directories(include)

# Declare a C++ executable
add_executable(obstacle_publisher src/GaussianBeliefTrees.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(obstacle_publisher
  rclcpp
  visualization_msgs
  geometry_msgs
  agent_helpers
)
target_link_libraries(obstacle_publisher
  yaml-cpp
)

# Install the executable
install(TARGETS
  obstacle_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Macro for ament package
ament_package()
