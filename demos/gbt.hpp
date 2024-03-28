/* Author: Qi Heng Ho
 *
 * Description:
 */

#ifndef GAUSSIAN_BELIEF_TREES_
#define GAUSSIAN_BELIEF_TREES_

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
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

#include <ompl/tools/debug/Profiler.h>
#include "ompl/tools/benchmark/Benchmark.h"
#include "StatePropagators/SimpleStatePropagator.h"
#include "ValidityCheckers/StateValidityCheckerBlackmore.h" 
#include "StatePropagators/SimpleStatePropagator.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RealVectorBeliefSpace.h"
#include "OptimizationObjectives/state_cost_objective.hpp"
#include "OptimizationObjectives/expected_cost_objective.hpp"
#include "Planners/SSBT.hpp"
#include "Scene/Scene.h"
#include "System/System.h"

#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

class GaussianBeliefTrees {
  public:
        GaussianBeliefTrees(const std::string& scene_config, const std::string& system_config);
        // ~GaussianBeliefTrees();
        ob::StateSpacePtr constructCSpace();
        void boundCSpace(ob::StateSpacePtr c_space);
        oc::ControlSpacePtr constructCtrlSpace(ob::StateSpacePtr c_space);
        void boundCtrlSpace(oc::ControlSpacePtr ctrl_space);
        void planWithSimpleSetup();
        void solve(ob::PlannerPtr planner);
        void SaveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string pathstring);


  private:
    std::string scene_id_;
    double planning_depth_, watchdog_period_, solving_time_, discretisation_time_, min_control_duration_, max_control_duration_, p_safe_, goal_tolerance_, confidence_level_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_, planning_bounds_z_, start_configuration_, goal_configuration_, controller_parameters_, surge_bounds_, heave_bounds_, forward_acceleration_bounds_, turning_rate_bounds_, heave_acceleration_bounds_, system_noise_;

    Eigen::Matrix<double, 2, 2, Eigen::DontAlign> initial_covariance_;

    // OMPL
    ob::StateValidityCheckerPtr om_stat_val_check_;
    oc::SimpleSetupPtr simple_setup_;
    ob::StateSpacePtr c_space_;

    std::vector<const ob::State *> path_states_;
    std::vector<const oc::Control *> path_controls_;

    Scene scene_;
    System system_;
};

#endif