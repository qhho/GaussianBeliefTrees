#ifndef BARRIER_RRT_MAIN_HPP
#define BARRIER_RRT_MAIN_HPP

#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include "Planners/mod_rrt.hpp"
#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "Spaces/RNBeliefSpace.h"
#include "StatePropagators/SimpleStatePropagator.h"

using namespace ompl;
using namespace ompl::base;
using namespace ompl::control;

class BarrierRRTMain
{
public:
    BarrierRRTMain();
    void planWithBarrierRRT();
    void loadConfig(const std::string& config_file);
    void loadScene(const std::string& scene_file);
    
    // Make scene_name_ public so it can be accessed from main
    std::string scene_name_;
    
private:
    // Configuration parameters
    std::vector<double> planning_bounds_x_;
    std::vector<double> planning_bounds_y_;
    std::vector<double> start_configuration_;
    std::vector<double> goal_configuration_;
    Eigen::MatrixXd initial_covariance_;
    
    // System parameters
    Eigen::MatrixXd A_, B_, K_, G_, Q_;
    double dt_;
    
    // Planner parameters
    double planning_time_;
    double goal_bias_;
    double sampling_bias_;
    bool intermediate_states_;
    std::vector<int> control_duration_;
    
    // Barrier parameters
    double risk_threshold_;
    int time_steps_;
    double step_duration_;
    
    // Scene parameters
    std::vector<Eigen::VectorXd> obstacle_constraints_a_;
    std::vector<double> obstacle_constraints_gamma_;
    
    // Helper methods
    void setupBarrierConstraints();
    void saveSolutionPath(const PathControl& path_control, 
                         const StateSpacePtr& space, 
                         const std::string& filepath);
};

#endif // BARRIER_RRT_MAIN_HPP 