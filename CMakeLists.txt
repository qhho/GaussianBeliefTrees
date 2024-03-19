cmake_minimum_required(VERSION 2.8.3)
project(gbt)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14 -march=native -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize -mavx2 -mfma -fomit-frame-pointer -fpermissive")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS})


add_library (rrbt_lib
    src/Spaces/RealVectorBeliefSpace.cpp
    src/Spaces/R2BeliefSpace.cpp
    src/Spaces/R2BeliefSpaceEuclidean.cpp
    src/Spaces/R3BeliefSpace.cpp
    scenes/scenereplanning.cpp
    scenes/scenereplanning2.cpp
    src/StatePropagators/SimpleStatePropagator.cpp
    src/StatePropagators/SimpleStatePropagatorfixedK.cpp
    src/StatePropagators/3DSimpleStatePropagator.cpp
    src/StatePropagators/3DSimpleStatePropagatorfixedK.cpp
    src/StatePropagators/2DUnicyclePropagator.cpp
    src/StatePropagators/2DUnicyclePropagatorfixedK.cpp
    src/StatePropagators/3DUnicyclePropagator.cpp
    src/StatePropagators/3DUnicyclePropagatorfixedK.cpp
    src/OptimizationObjectives/expected_cost_objective.cpp
    src/OptimizationObjectives/state_cost_objective.cpp
    src/ValidityCheckers/state_validity_checker_pcc_blackmore.cpp
    # src/ValidityCheckers/state_validity_checker_pcc_blackmore_euclidean.cpp
    src/Benchmarking/MyBenchmark.cpp
    src/Benchmarking/MyBenchmarkRRBT.cpp
    src/Planners/SSBT.cpp
    src/Planners/mod_rrt.cpp
    src/Planners/rrbt.cpp
    src/Planners/rrbt_unicycle.cpp
    # src/Planners/rrbt_fixedwing.cpp
    src/Scene/Scene.cpp
    src/StateSamplers/BeliefStateSampler.cpp
    src/StateSamplers/CompoundBeliefStateSampler.cpp
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

# add_executable (main src/simple_main.cpp)

# target_link_libraries (main
#     rrbt_lib
#     ${OMPL_LIBRARIES}
#     ${Boost_LIBRARIES}
# )

add_executable (varyK benchmarks/benchmark_varyK.cpp)

target_link_libraries (varyK
    rrbt_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)

add_executable (rrbt benchmarks/benchmark_rrbt.cpp)

target_link_libraries (rrbt
    rrbt_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)

add_executable (fixedK benchmarks/benchmark_fixedK.cpp)

target_link_libraries (fixedK
    rrbt_lib
    ${OMPL_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
)