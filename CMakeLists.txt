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


add_library (rrbt_lib
    src/Spaces/RealVectorBeliefSpace.cpp
    src/Spaces/R2BeliefSpace.cpp
    src/Spaces/R2BeliefSpaceEuclidean.cpp
    scenes/scene3.cpp
    scenes/scene4.cpp
    scenes/scenereplanning.cpp
    scenes/scenereplanning2.cpp
    src/StatePropagators/SimpleStatePropagator.cpp
    src/StatePropagators/SimpleStatePropagatorEuclidean.cpp
    src/OptimizationObjectives/state_cost_objective.cpp
    src/ValidityCheckers/state_validity_checker_pcc_blackmore.cpp
    src/ValidityCheckers/state_validity_checker_pcc_blackmore_euclidean.cpp
    src/Benchmarking/MyBenchmark.cpp
    src/Benchmarking/MyBenchmarkRRBT.cpp
    src/Planners/SSBT.cpp
    src/Planners/mod_rrt.cpp
    src/Scene/Scene.cpp
    src/System/System.cpp
    src/ValidityCheckers/StateValidityCheckerBlackmore.cpp
)


target_link_libraries (rrbt_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    yaml-cpp
)

target_include_directories(rrbt_lib
    PUBLIC
    ${OMPL_INCLUDE_DIRS}
)

add_executable (main src/simple_main.cpp)

target_link_libraries (main
    rrbt_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)


# add_executable (euc src/euclidean_main.cpp)

# target_link_libraries (euc
#     rrbt_lib
#     ${OMPL_LIBRARIES}
#     ${Boost_LIBRARIES}
# )

# add_executable(createobstacles scenario1_create.cpp)