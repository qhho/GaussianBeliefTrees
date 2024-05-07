cmake_minimum_required(VERSION 3.5)
project(agent_gbt)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")

# Find packages
find_package(gbt_lib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)  # Ensure ament_cmake is found
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# Include directories
include_directories(
  include
  ${GBT_LIB_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(agent_gbt src/GBT.cpp)

# Link libraries using ament
ament_target_dependencies(agent_gbt
  rclcpp
  visualization_msgs
  geometry_msgs
  ompl
  Boost
)

# Link against the gbt library
target_link_libraries(agent_gbt
  gbt_lib
  yaml-cpp
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# Install the executable
install(TARGETS agent_gbt
  DESTINATION lib/${PROJECT_NAME}
)

# Additional ament configuration
ament_package()