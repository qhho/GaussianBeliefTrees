cmake_minimum_required(VERSION 3.5)
project(gbt)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14 -march=native -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize -mavx2 -mfma -fomit-frame-pointer -fpermissive")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++17")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "stdc++fs")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "c++fs")
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS}  ${CMAKE_SOURCE_DIR}/src)


add_library (gbt_lib
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


target_link_libraries (gbt_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    yaml-cpp
)

target_include_directories(gbt_lib
    PUBLIC
    ${OMPL_INCLUDE_DIRS}
)

add_executable (main demos/gbt.cpp)

target_link_libraries (main
    gbt_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)