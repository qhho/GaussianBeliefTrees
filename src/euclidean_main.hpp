/* Author: Qi Heng Ho
 *
 * Description:
 */

#ifndef EUCLIDEAN_MAIN_
#define EUCLIDEAN_MAIN_

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/bind.hpp>
#include <math.h>

// OMPL
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
// #include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

#include <ompl/tools/debug/Profiler.h>
#include "ompl/tools/benchmark/Benchmark.h"

// path controller
//#include <epa_path_ctrl/PathState.h>
//#include <epa_path_ctrl/Path.h>

#include "StatePropagators/SimpleStatePropagator.h"
#include "Planners/SSBT.hpp"
// #include "Planners/mod_rrt.hpp"
// #include "mod_sst.hpp"
// #include "Planners/rrg.hpp"

#include <limits>
#include "Scene/Scene.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class EuclideanMain {
  public:
    EuclideanMain(const std::string& scene_config);
    // ~EuclideanMain();

    ob::StateSpacePtr constructCSpace();
    void boundCSpace(ob::StateSpacePtr c_space);

    oc::ControlSpacePtr constructCtrlSpace(ob::StateSpacePtr c_space);
    void boundCtrlSpace(oc::ControlSpacePtr ctrl_space);

    void planWithSimpleSetup();
    void visualizeProblemDefinition();

    void replacePath(og::PathGeometric geopath);
    void solve(ob::PlannerPtr planner);

    void SaveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string pathstring);

    Scene scene_;


  private:
    std::string scene_id_;
    bool SVC_first_iteration_;
    double planning_depth_, watchdog_period_, solving_time_, discretisation_time_, min_control_duration_, max_control_duration_, p_safe_, goal_tolerance_, confidence_level_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_, planning_bounds_z_, start_configuration_, goal_configuration_, controller_parameters_, surge_bounds_, heave_bounds_, forward_acceleration_bounds_, turning_rate_bounds_, heave_acceleration_bounds_, system_noise_;

    Eigen::Matrix<double, 2, 2, Eigen::DontAlign> initial_covariance_;

    // OMPL
    ob::StateValidityCheckerPtr om_stat_val_check_;
    oc::SimpleSetupPtr simple_setup_;
    ob::StateSpacePtr c_space_;

    // LBKS
    double last_best_known_solution_remaining_length_;
    double last_best_known_solution_length_;
    std::vector<double> last_best_known_solution_accumulative_remaining_length_;
    std::vector<const ob::State*> last_best_known_solution_states_;

    // safety
    int minimum_vertices;
    int iterations_vehicle_stuck;
    int iterations_vehicle_stuck_limit;

    std::vector<const ob::State *> path_states_;
    std::vector<const oc::Control *> path_controls_;

    //
    int svc_method_;
};

#endif