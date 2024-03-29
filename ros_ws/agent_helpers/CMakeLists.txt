cmake_minimum_required(VERSION 3.5)
project(agent_helpers)

# Default to C99 and C++14
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)


# Include directories
include_directories(include
${YAML_CPP_INCLUDE_DIR}
)

# Add a library
add_library(agent_helpers
  src/Scene.cpp
  src/System.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(agent_helpers
  rclcpp 
  Eigen3 
  geometry_msgs
)

target_link_libraries(agent_helpers ${YAML_CPP_LIBRARIES})

# Install header files and library
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(TARGETS agent_helpers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies and library
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(agent_helpers)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package configuration
ament_package()
