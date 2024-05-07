cmake_minimum_required(VERSION 3.5)
project(gbt VERSION 1.0.0)

# Set compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")

# Check compiler version for filesystem library dependency
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "stdc++fs")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "c++fs")
endif()

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

# Define the library
add_library(gbt_lib
    src/Spaces/RealVectorBeliefSpace.cpp
    src/Spaces/R2BeliefSpace.cpp
    src/StatePropagators/SimpleStatePropagator.cpp
    src/OptimizationObjectives/state_cost_objective.cpp
    src/Benchmarking/MyBenchmark.cpp
    src/Planners/SSBT.cpp
    src/Planners/mod_rrt.cpp
    src/Scene/Scene.cpp
    src/System/System.cpp
    src/ValidityCheckers/StateValidityCheckerBlackmore.cpp
)

# Link libraries
target_link_libraries(gbt_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    yaml-cpp
)

# Specify include directories
target_include_directories(gbt_lib
    PUBLIC
    ${OMPL_INCLUDE_DIRS}
)

# Define the executable
add_executable(main demos/gbt.cpp)

# Link the executable with libraries
target_link_libraries(main
    gbt_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)

# Installation rules for targets and headers
install(TARGETS gbt_lib main
    EXPORT gbt_libTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Export gbt_lib as a CMake package
install(EXPORT gbt_libTargets
    FILE gbt_libTargets.cmake
    NAMESPACE gbt_lib::
    DESTINATION lib/cmake/gbt_lib
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    gbt_libConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/gbt_libConfig.cmake"
    INSTALL_DESTINATION lib/cmake/gbt_lib
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/gbt_libConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/gbt_libConfigVersion.cmake"
    DESTINATION lib/cmake/gbt_lib
)