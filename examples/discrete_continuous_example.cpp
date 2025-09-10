#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>

// OMPL
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// headers
#include "ValidityCheckers/state_validity_checker_pcc_blackmore.hpp"
#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "Planners/mod_rrt.hpp"
#include "StatePropagators/SimpleStatePropagator.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RNBeliefSpace.h"
#include "OptimizationObjectives/state_cost_objective.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

// Custom goal region for belief states
class BeliefGoalRegion : public GoalRegion
{
public:
    BeliefGoalRegion(const ompl::control::SpaceInformationPtr &si, const State *goal_state, double threshold = 0.5)
        : GoalRegion(si), goal_state_(si->cloneState(goal_state)), threshold_(threshold)
    {
    }
    
    virtual ~BeliefGoalRegion()
    {
        si_->freeState(goal_state_);
    }
    
    virtual double distanceGoal(const State *state) const override
    {
        auto state_belief = state->as<RNBeliefSpace::StateType>();
        auto goal_belief = goal_state_->as<RNBeliefSpace::StateType>();
        
        double dx = state_belief->getX() - goal_belief->getX();
        double dy = state_belief->getY() - goal_belief->getY();
        
        if (std::sqrt(dx*dx + dy*dy) < threshold_)
        {
            return 0.0;
        }
        
        return std::sqrt(dx*dx + dy*dy);
    }

private:
    State *goal_state_;
    double threshold_;
};

class DiscreteContinuousExample
{
public:
    DiscreteContinuousExample(const std::string& config_file);
    void loadConfig(const std::string& config_file);
    void loadScene(const std::string& scene_file);
    void setupBarrierConstraints();
    void planWithDiscreteTime();
    void validateWithContinuousTime();
    void saveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string stringpath);
    void saveIntermediateStates(const std::vector<ob::State*>& intermediate_states, 
                               const std::vector<oc::Control*>& intermediate_controls,
                               const std::vector<double>& intermediate_durations,
                               std::string stringpath);
    void generateIntermediateStates(const oc::PathControl& path_control, 
                                  std::vector<ob::State*>& intermediate_states,
                                  std::vector<oc::Control*>& intermediate_controls,
                                  std::vector<double>& intermediate_durations);
    bool validateTrajectoryStepByStep(const std::vector<ob::State*>& intermediate_states,
                                     const std::vector<oc::Control*>& intermediate_controls,
                                     const std::vector<double>& intermediate_durations,
                                     std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker,
                                     std::vector<ob::State*>& valid_states,
                                     std::vector<oc::Control*>& valid_controls,
                                     std::vector<double>& valid_durations,
                                     int& failure_index);

    // Helper methods for trajectory validation (same as in continuous_rrt_with_trajectory_checking.cpp)
    std::vector<ompl::base::State*> propagateWhileValidWithTrajectoryChecking(
        const ompl::base::State* state,
        const ompl::control::Control* control,
        unsigned int steps,
        std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker,
        const ompl::control::SpaceInformationPtr& si) const;
    
    bool checkTrajectoryValidityAtStep(
        const ompl::base::State *current_state, 
        const ompl::control::Control *control, 
        double step_duration,
        std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker) const;

private:
    // Configuration parameters
    std::vector<double> planning_bounds_x_;
    std::vector<double> planning_bounds_y_;
    std::vector<double> start_configuration_;
    std::vector<double> goal_configuration_;
    Eigen::MatrixXd initial_covariance_;
    std::string scene_name_;
    double solving_time_;
    double Q_noise_;
    double R_noise_;
    double R_bad_;
    double K_default_;
    
    // System matrices for continuous time validation
    Eigen::MatrixXd A_, B_, K_, G_, Q_;
    double dt_;
    
    // Barrier parameters
    double risk_threshold_;
    int time_steps_;
    double step_duration_;
    
    // Obstacle constraints
    std::vector<std::vector<Eigen::VectorXd>> obstacle_a_lists_;
    std::vector<std::vector<double>> obstacle_gamma_lists_;
    
    // Solution data
    std::vector<ob::State*> path_states_;
    std::vector<oc::Control*> path_controls_;
    std::shared_ptr<oc::PathControl> solution_path_;
    bool solution_found_;
};

DiscreteContinuousExample::DiscreteContinuousExample(const std::string& config_file)
    : solution_path_(nullptr), solution_found_(false)
{
    // Initialize with default values
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_configuration_.resize(2);
    goal_configuration_.resize(2);

    if (!config_file.empty()) {
        loadConfig(config_file);
    } else {
        // Default values if no config file
        planning_bounds_x_[0] = 0.0;
        planning_bounds_x_[1] = 100.0;
        planning_bounds_y_[0] = 0.0;
        planning_bounds_y_[1] = 100.0;
        start_configuration_[0] = 10.0;
        start_configuration_[1] = 10.0;
        goal_configuration_[0] = 90.0;
        goal_configuration_[1] = 90.0;
        initial_covariance_ = 1.0*Eigen::MatrixXd::Identity(2, 2);
        scene_name_ = "scene3";
        solving_time_ = 60.0;
        Q_noise_ = 0.2;
        R_noise_ = 0.1;
        R_bad_ = 5.0;
        K_default_ = 0.9;
        
        // Default system matrices for continuous time
        A_ = Eigen::MatrixXd::Identity(2, 2);
        B_ = Eigen::MatrixXd::Identity(2, 2);
        K_ = Eigen::MatrixXd::Zero(2, 2);
        G_ = Eigen::MatrixXd::Identity(2, 2);
        Q_ = Q_noise_ * Eigen::MatrixXd::Identity(2, 2);
        dt_ = 0.1;
        
        // Default barrier parameters
        risk_threshold_ = 0.01;
        time_steps_ = 50;
        step_duration_ = 0.002;
    }
}

void DiscreteContinuousExample::loadConfig(const std::string& config_file)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    
    // Load environment bounds
    std::string x_bounds = pt.get<std::string>("Environment.x_bounds");
    std::string y_bounds = pt.get<std::string>("Environment.y_bounds");
    
    // Parse bounds (format: "min,max")
    size_t comma_pos = x_bounds.find(',');
    planning_bounds_x_[0] = std::stod(x_bounds.substr(0, comma_pos));
    planning_bounds_x_[1] = std::stod(x_bounds.substr(comma_pos + 1));
    
    comma_pos = y_bounds.find(',');
    planning_bounds_y_[0] = std::stod(y_bounds.substr(0, comma_pos));
    planning_bounds_y_[1] = std::stod(y_bounds.substr(comma_pos + 1));
    
    // Load start and goal configurations
    std::string start_config = pt.get<std::string>("Environment.start_configuration");
    std::string goal_config = pt.get<std::string>("Environment.goal_configuration");
    
    comma_pos = start_config.find(',');
    start_configuration_[0] = std::stod(start_config.substr(0, comma_pos));
    start_configuration_[1] = std::stod(start_config.substr(comma_pos + 1));
    
    comma_pos = goal_config.find(',');
    goal_configuration_[0] = std::stod(goal_config.substr(0, comma_pos));
    goal_configuration_[1] = std::stod(goal_config.substr(comma_pos + 1));
    
    // Load system parameters
    double initial_cov = pt.get<double>("System.initial_covariance");
    initial_covariance_ = initial_cov * Eigen::MatrixXd::Identity(2, 2);
    
    // Load system noise parameters
    Q_noise_ = pt.get<double>("System.Q");
    R_noise_ = pt.get<double>("System.R");
    R_bad_ = pt.get<double>("System.R_bad");
    K_default_ = pt.get<double>("System.K_default");
    
    // Set system matrices for continuous time
    A_ = Eigen::MatrixXd::Identity(2, 2);
    B_ = Eigen::MatrixXd::Identity(2, 2);
    K_ = Eigen::MatrixXd::Zero(2, 2);
    G_ = Eigen::MatrixXd::Identity(2, 2);
    Q_ = Q_noise_ * Eigen::MatrixXd::Identity(2, 2);
    dt_ = 0.1;
    
    // Load scene name
    scene_name_ = pt.get<std::string>("Scene.scene");

    // Load planner parameters
    solving_time_ = pt.get<double>("Planner.planning_time");
    
    // Load barrier parameters if available
    try {
        risk_threshold_ = pt.get<double>("Barrier.risk_threshold");
        time_steps_ = pt.get<int>("Barrier.time_steps");
        step_duration_ = pt.get<double>("Barrier.step_duration");
    } catch (...) {
        // Use defaults if not specified
        risk_threshold_ = 0.01;
        time_steps_ = 50;
        step_duration_ = 0.002;
    }
    
    // Load scene if specified
    if (!scene_name_.empty()) {
        loadScene(scene_name_);
    }
}

void DiscreteContinuousExample::loadScene(const std::string& scene_file)
{
    std::cout << "Loading scene from: " << scene_file << std::endl;
    
    YAML::Node config = YAML::LoadFile(scene_file);
    
    if (config["scene"]["obstacles"]) {
        std::cout << "Found obstacles in scene file" << std::endl;
        
        for (const auto& obstacle : config["scene"]["obstacles"]) {
            std::cout << "Processing obstacle..." << std::endl;
            
            // Create vectors for this specific obstacle
            std::vector<Eigen::VectorXd> obstacle_a_list;
            std::vector<double> obstacle_gamma_list;
            
            // Check if it's a circle obstacle (has type field)
            if (obstacle["type"]) {
                std::string obstacle_type = obstacle["type"].as<std::string>();
                std::cout << "Circle obstacle type: " << obstacle_type << std::endl;
                
                if (obstacle_type == "circle") {
                    double center_x = obstacle["center_x"].as<double>();
                    double center_y = obstacle["center_y"].as<double>();
                    double radius = obstacle["radius"].as<double>();
                    int num_constraints = obstacle["num_constraints"].as<int>();
                    
                    // Generate half-space constraints for circle
                    for (int i = 0; i < num_constraints; ++i) {
                        double angle = 2.0 * M_PI * i / num_constraints;
                        Eigen::VectorXd a_obs(2);
                        // Point AWAY from the center (positive direction)
                        a_obs << cos(angle), sin(angle);
                        obstacle_a_list.push_back(a_obs);
                        // The constraint should be: a^T * point >= a^T * center + radius
                        obstacle_gamma_list.push_back(center_x * cos(angle) + center_y * sin(angle) + radius);
                    }
                    
                    std::cout << "Added " << num_constraints << " half-space constraints for circle" << std::endl;
                }
            }
            // Check if it's a rectangular obstacle (has fx, tx, fy, ty fields)
            else if (obstacle["fx"] && obstacle["tx"] && obstacle["fy"] && obstacle["ty"]) {
                std::cout << "Processing rectangular obstacle" << std::endl;
                
                double fx = obstacle["fx"].as<double>();
                double tx = obstacle["tx"].as<double>();
                double fy = obstacle["fy"].as<double>();
                double ty = obstacle["ty"].as<double>();
                
                std::cout << "Rectangle: (" << fx << "," << fy << ") to (" << tx << "," << ty << ")" << std::endl;
                
                // Create 4 half-space constraints for rectangle
                // Left: x >= fx
                Eigen::VectorXd a1(2);
                a1 << -1.0, 0.0;
                obstacle_a_list.push_back(a1);
                obstacle_gamma_list.push_back(-fx);
                
                // Right: x <= tx
                Eigen::VectorXd a2(2);
                a2 << 1.0, 0.0;
                obstacle_a_list.push_back(a2);
                obstacle_gamma_list.push_back(tx);
                
                // Bottom: y >= fy
                Eigen::VectorXd a3(2);
                a3 << 0.0, -1.0;
                obstacle_a_list.push_back(a3);
                obstacle_gamma_list.push_back(-fy);
                
                // Top: y <= ty
                Eigen::VectorXd a4(2);
                a4 << 0.0, 1.0;
                obstacle_a_list.push_back(a4);
                obstacle_gamma_list.push_back(ty);
                
                std::cout << "Added 4 half-space constraints for rectangle" << std::endl;
            }
            else {
                std::cout << "Unknown obstacle type, skipping..." << std::endl;
                continue; // Skip this obstacle
            }
            
            // Add this obstacle to the multiple obstacles list
            if (!obstacle_a_list.empty()) {
                obstacle_a_lists_.push_back(obstacle_a_list);
                obstacle_gamma_lists_.push_back(obstacle_gamma_list);
                std::cout << "Added obstacle with " << obstacle_a_list.size() << " constraints" << std::endl;
            }
        }
    }
    else {
        std::cout << "No obstacles found in scene file" << std::endl;
    }
    
    std::cout << "Total obstacles loaded: " << obstacle_a_lists_.size() << std::endl;
    for (size_t i = 0; i < obstacle_a_lists_.size(); ++i) {
        std::cout << "  Obstacle " << i << ": " << obstacle_a_lists_[i].size() << " constraints" << std::endl;
    }
}

void DiscreteContinuousExample::setupBarrierConstraints()
{
    // Create the final multiple obstacles list
    std::vector<std::vector<Eigen::VectorXd>> final_obstacle_a_lists;
    std::vector<std::vector<double>> final_obstacle_gamma_lists;
    
    // Add each boundary constraint as a separate obstacle (AND logic between boundaries)
    
    // Left boundary: x >= 0
    std::vector<Eigen::VectorXd> left_boundary;
    std::vector<double> left_gamma;
    Eigen::VectorXd a1(2);
    a1 << 1.0, 0.0;
    left_boundary.push_back(a1);
    left_gamma.push_back(0.0);
    final_obstacle_a_lists.push_back(left_boundary);
    final_obstacle_gamma_lists.push_back(left_gamma);
    
    // Right boundary: x <= max_x
    std::vector<Eigen::VectorXd> right_boundary;
    std::vector<double> right_gamma;
    Eigen::VectorXd a2(2);
    a2 << -1.0, 0.0;
    right_boundary.push_back(a2);
    right_gamma.push_back(-planning_bounds_x_[1]);
    final_obstacle_a_lists.push_back(right_boundary);
    final_obstacle_gamma_lists.push_back(right_gamma);
    
    // Bottom boundary: y >= 0
    std::vector<Eigen::VectorXd> bottom_boundary;
    std::vector<double> bottom_gamma;
    Eigen::VectorXd a3(2);
    a3 << 0.0, 1.0;
    bottom_boundary.push_back(a3);
    bottom_gamma.push_back(0.0);
    final_obstacle_a_lists.push_back(bottom_boundary);
    final_obstacle_gamma_lists.push_back(bottom_gamma);
    
    // Top boundary: y <= max_y
    std::vector<Eigen::VectorXd> top_boundary;
    std::vector<double> top_gamma;
    Eigen::VectorXd a4(2);
    a4 << 0.0, -1.0;
    top_boundary.push_back(a4);
    top_gamma.push_back(-planning_bounds_y_[1]);
    final_obstacle_a_lists.push_back(top_boundary);
    final_obstacle_gamma_lists.push_back(top_gamma);
    
    // Add all scene obstacles (each as a separate obstacle)
    final_obstacle_a_lists.insert(final_obstacle_a_lists.end(), 
                                  obstacle_a_lists_.begin(), 
                                  obstacle_a_lists_.end());
    final_obstacle_gamma_lists.insert(final_obstacle_gamma_lists.end(), 
                                      obstacle_gamma_lists_.begin(), 
                                      obstacle_gamma_lists_.end());
    
    // Store the final combined list
    obstacle_a_lists_ = final_obstacle_a_lists;
    obstacle_gamma_lists_ = final_obstacle_gamma_lists;
    
    std::cout << "Setup complete: " << obstacle_a_lists_.size() << " obstacles total" << std::endl;
    std::cout << "  - Boundary constraints: 4 separate obstacles (AND logic)" << std::endl;
    std::cout << "  - Scene obstacles: " << (obstacle_a_lists_.size() - 4) << " obstacles" << std::endl;
}

void DiscreteContinuousExample::planWithDiscreteTime()
{
    std::cout << "Starting discrete time planning..." << std::endl;
    std::cout << "Scene: " << scene_name_ << std::endl;
    
    //=======================================================================
    // Instantiate the state space (RNBeliefSpace)
    //=======================================================================
    ob::StateSpacePtr space = ob::StateSpacePtr(new RNBeliefSpace(2, initial_covariance_));
    
    // Set the bounds for the R^2 part of RNBeliefSpace
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, planning_bounds_x_[0]);
    bounds_se2.setHigh(0, planning_bounds_x_[1]);
    bounds_se2.setLow(1, planning_bounds_y_[0]);
    bounds_se2.setHigh(1, planning_bounds_y_[1]);
    
    space->as<RNBeliefSpace>()->setBounds(bounds_se2);
    
    //=======================================================================
    // Instantiate the control space
    //=======================================================================
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -1.0);
    bounds.setHigh(0, 1.0);
    bounds.setLow(1, -1.0);
    bounds.setHigh(1, 1.0);
    bounds.setLow(2, 0.0);
    bounds.setHigh(2, 0.6);
    
    cspace->setBounds(bounds);
    
    //=======================================================================
    // Define space information
    //=======================================================================
    oc::SpaceInformationPtr si(new oc::SpaceInformation(space, cspace));
    
    // Set minimum and maximum duration of control action
    si->setMinMaxControlDuration(1, 5);
    si->setPropagationStepSize(0.1);

    //=======================================================================
    // Create a planner for the defined space
    //=======================================================================
    double goal_bias_ = 0.05;
    double sampling_bias_ = 0.20;

    ob::PlannerPtr planner;
    planner = ob::PlannerPtr(new oc::mod_RRT(si));
    planner->as<oc::mod_RRT>()->setGoalBias(goal_bias_);
    planner->as<oc::mod_RRT>()->setSamplingBias(sampling_bias_);
    planner->as<oc::mod_RRT>()->setDistanceFunction(1); //1 is for wasserstein
    
    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    ob::ScopedState<> start(space);
    start[0] = double(start_configuration_[0]); //x
    start[1] = double(start_configuration_[1]); //y

    ob::ScopedState<> goal(space);
    goal[0] = double(goal_configuration_[0]); //x
    goal[1] = double(goal_configuration_[1]); //y

    //=======================================================================
    // Set the propagation routine for this space
    //=======================================================================
    std::vector<std::vector<double>> measurement_regions = {
        {planning_bounds_x_[0], planning_bounds_x_[1]}, 
        {planning_bounds_y_[0], planning_bounds_y_[1]}
    };
    si->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si, Q_noise_, R_noise_, R_bad_, K_default_, measurement_regions)));

    //=======================================================================
    // Set state validity checker (discrete time)
    //=======================================================================
    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore(scene_name_, si, 0.99, 0));
    si->setStateValidityChecker(om_stat_val_check);

    si->setup();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<BeliefGoalRegion>(si, goal.get(), 10.0));

    pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));
    pdef->getOptimizationObjective()->setCostThreshold(ob::Cost(45.0));
    planner->setProblemDefinition(pdef);

    planner->setup();

    std::cout << "Starting planning..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Solve the problem
    ob::PlannerStatus status = planner->solve(solving_time_);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Planning completed in " << duration.count() << " ms" << std::endl;

    if (status == ob::PlannerStatus::EXACT_SOLUTION || status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "Solution found!" << std::endl;
        if (status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
            std::cout << "  (Approximate solution)" << std::endl;
        }
        
        // Get the solution path
        const ompl::base::PathPtr &path = planner->getProblemDefinition()->getSolutionPath();
        solution_path_ = std::make_shared<oc::PathControl>(static_cast<oc::PathControl&>(*path));
        solution_found_ = true;
        
        // Save the discrete time solution
        saveSolutionPath(*solution_path_, space, "solution_discrete.csv");
        
        std::cout << "Discrete time solution saved to solution_discrete.csv" << std::endl;
    }
    else
    {
        std::cout << "No solution found." << std::endl;
        std::cout << "Status: " << status << std::endl;
        solution_found_ = false;
    }
}

void DiscreteContinuousExample::generateIntermediateStates(const oc::PathControl& path_control, 
                                                          std::vector<ob::State*>& intermediate_states,
                                                          std::vector<oc::Control*>& intermediate_controls,
                                                          std::vector<double>& intermediate_durations)
{
    std::cout << "Generating intermediate states using SAME validation as planner..." << std::endl;
    
    // Get the states and controls from the path
    std::vector<ob::State*> path_states = const_cast<oc::PathControl&>(path_control).getStates();
    std::vector<oc::Control*> path_controls = const_cast<oc::PathControl&>(path_control).getControls();
    
    // Clear output vectors
    intermediate_states.clear();
    intermediate_controls.clear();
    intermediate_durations.clear();
    
    // Get space information from the path - this is oc::SpaceInformation
    auto si = std::dynamic_pointer_cast<oc::SpaceInformation>(path_control.getSpaceInformation());
    
    // Create the barrier trajectory validity checker (SAME as planner)
    auto validity_checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    validity_checker->setSystemMatrices(A_, B_, K_, G_, Q_);
    validity_checker->setMultipleObstacles(obstacle_a_lists_, obstacle_gamma_lists_, risk_threshold_);
    validity_checker->setTimeParameters(time_steps_, step_duration_);
    
    // For each control segment, use the SAME validation approach as the planner
    for (size_t i = 0; i < path_controls.size(); ++i) {
        double duration = path_control.getControlDuration(i);
        int num_steps = static_cast<int>(duration / dt_);
        
        std::cout << "Control " << i << ": duration = " << duration 
                  << ", generating " << num_steps << " intermediate states" << std::endl;
        
        // Use the SAME validation approach as propagateWhileValidWithTrajectoryChecking
        std::vector<ompl::base::State*> pstates = propagateWhileValidWithTrajectoryChecking(
            path_states[i], path_controls[i], num_steps, validity_checker, si);
        
        std::cout << "  Actual steps propagated: " << pstates.size() << std::endl;
        
        // Add all propagated states to our intermediate states
        for (size_t j = 0; j < pstates.size(); ++j) {
            intermediate_states.push_back(pstates[j]);
            intermediate_controls.push_back(path_controls[i]);
            intermediate_durations.push_back(dt_);
        }
    }
    
    std::cout << "Generated " << intermediate_states.size() << " intermediate states using SAME validation as planner" << std::endl;
}

bool DiscreteContinuousExample::validateTrajectoryStepByStep(const std::vector<ob::State*>& intermediate_states,
                                                             const std::vector<oc::Control*>& intermediate_controls,
                                                             const std::vector<double>& intermediate_durations,
                                                             std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker,
                                                             std::vector<ob::State*>& valid_states,
                                                             std::vector<oc::Control*>& valid_controls,
                                                             std::vector<double>& valid_durations,
                                                             int& failure_index)
{
    std::cout << "Validating trajectory step by step..." << std::endl;
    
    valid_states.clear();
    valid_controls.clear();
    valid_durations.clear();
    failure_index = -1;
    
    // Start with the first state (always valid)
    valid_states.push_back(intermediate_states[0]);
    valid_controls.push_back(intermediate_controls[0]);
    valid_durations.push_back(intermediate_durations[0]);
    
    // Check each subsequent state
    for (size_t i = 1; i < intermediate_states.size(); ++i) {
        // Create a trajectory from start to current state
        std::vector<oc::Control*> current_controls(intermediate_controls.begin(), intermediate_controls.begin() + i);
        std::vector<double> current_durations(intermediate_durations.begin(), intermediate_durations.begin() + i);
        
        // Validate this partial trajectory
        bool is_valid = validity_checker->isValidTrajectory(intermediate_states[0], current_controls, current_durations);
        
        if (is_valid) {
            // This state is valid, add it to the valid trajectory
            valid_states.push_back(intermediate_states[i]);
            valid_controls.push_back(intermediate_controls[i]);
            valid_durations.push_back(intermediate_durations[i]);
        } else {
            // This state is invalid, record the failure index
            failure_index = i;
            std::cout << "Trajectory becomes invalid at step " << i << " (state " << i << ")" << std::endl;
            break;
        }
    }
    
    bool is_fully_valid = (failure_index == -1);
    std::cout << "Step-by-step validation complete. Valid states: " << valid_states.size() 
              << " out of " << intermediate_states.size() << std::endl;
    
    return is_fully_valid;
}

void DiscreteContinuousExample::saveIntermediateStates(const std::vector<ob::State*>& intermediate_states, 
                                                      const std::vector<oc::Control*>& intermediate_controls,
                                                      const std::vector<double>& intermediate_durations,
                                                      std::string stringpath)
{
    std::ofstream output_file(stringpath);
    output_file << "x,y,sigma_trace,lambda_trace,control_x,control_y,control_duration" << std::endl;
    
    for (size_t i = 0; i < intermediate_states.size(); ++i) {
        auto state = intermediate_states[i];
        auto belief = state->as<RNBeliefSpace::StateType>();
        
        // Get control information
        auto control = intermediate_controls[i];
        auto control_values = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        double duration = intermediate_durations[i];
        
        output_file << belief->getX() << "," 
                   << belief->getY() << "," 
                   << belief->getSigma().trace() << "," 
                   << belief->getLambda().trace() << ","
                   << control_values[0] << ","
                   << control_values[1] << ","
                   << duration << std::endl;
    }
    
    output_file.close();
    std::cout << "Intermediate states saved to: " << stringpath << std::endl;
}

void DiscreteContinuousExample::validateWithContinuousTime()
{
    if (!solution_found_) {
        std::cout << "No solution to validate!" << std::endl;
        return;
    }
    
    std::cout << "Starting continuous time validation using SAME approach as planner..." << std::endl;
    
    // Setup barrier constraints
    setupBarrierConstraints();
    
    // Create a new state space for validation
    auto state_space = std::make_shared<RNBeliefSpace>(2, initial_covariance_);
    
    // Set bounds
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, planning_bounds_x_[0]);
    bounds_se2.setHigh(0, planning_bounds_x_[1]);
    bounds_se2.setLow(1, planning_bounds_y_[0]);
    bounds_se2.setHigh(1, planning_bounds_y_[1]);
    state_space->setBounds(bounds_se2);
    
    // Create control space
    auto control_space = std::make_shared<oc::RealVectorControlSpace>(state_space, 3);
    
    // Create space information
    auto si = std::make_shared<ompl::control::SpaceInformation>(state_space, control_space);
    
    // Generate intermediate states using the SAME validation approach as the planner
    std::vector<ob::State*> intermediate_states;
    std::vector<oc::Control*> intermediate_controls;
    std::vector<double> intermediate_durations;
    
    generateIntermediateStates(*solution_path_, intermediate_states, intermediate_controls, intermediate_durations);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Check if the trajectory was fully valid by comparing expected vs actual states
    // If any step failed validation, we'll have fewer states than expected
    std::vector<ob::State*> path_states = const_cast<oc::PathControl&>(*solution_path_).getStates();
    std::vector<oc::Control*> path_controls = const_cast<oc::PathControl&>(*solution_path_).getControls();
    
    // Calculate expected number of intermediate states
    int expected_states = 0;
    for (size_t i = 0; i < path_controls.size(); ++i) {
        double duration = solution_path_->getControlDuration(i);
        int num_steps = static_cast<int>(duration / dt_);
        expected_states += num_steps + 1; // +1 for the starting state of each segment
    }
    
    bool is_fully_valid = (intermediate_states.size() == expected_states);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Validation completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Expected states: " << expected_states << ", Actual states: " << intermediate_states.size() << std::endl;
    
    if (is_fully_valid) {
        std::cout << "✅ Trajectory is VALID according to continuous time barrier constraints!" << std::endl;
        // Save the full valid trajectory
        saveIntermediateStates(intermediate_states, intermediate_controls, intermediate_durations, "solution_continuous_valid.csv");
    } else {
        std::cout << "❌ Trajectory is INVALID according to continuous time barrier constraints!" << std::endl;
        std::cout << "Some steps failed validation during propagation!" << std::endl;
        // Save the valid portion of the trajectory (up to the failure point)
        saveIntermediateStates(intermediate_states, intermediate_controls, intermediate_durations, "solution_continuous_invalid.csv");
    }
    
    // Note: Don't free intermediate states here as they are managed by OMPL
    // The propagateWhileValid method returns states that are managed by the SpaceInformation
}

void DiscreteContinuousExample::saveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string stringpath)
{   
    std::ofstream outputsolution;
    outputsolution.open(stringpath, std::ios::out | std::ios::trunc);
    outputsolution << "x,y,sigma_trace,lambda_trace" << std::endl;
    
    std::vector<ob::State*> path_control_states;
    path_control_states = path_control.getStates();

    for (size_t i = 0; i < path_control_states.size(); i++)
    {
        ob::State *s = space->allocState();
        space->copyState(s, path_control_states[i]);
        path_states_.push_back(s);

        double x_pose = s->as<RNBeliefSpace::StateType>()->getX();
        double y_pose = s->as<RNBeliefSpace::StateType>()->getY();
        Mat sigma = s->as<RNBeliefSpace::StateType>()->getSigma();
        Mat lambda = s->as<RNBeliefSpace::StateType>()->getLambda();

        outputsolution << x_pose << ","  << y_pose << "," << sigma.trace() << "," << lambda.trace() << std::endl;
    }

    outputsolution.close();
}

// Helper method that implements the SAME validation logic as the planner
std::vector<ompl::base::State*> DiscreteContinuousExample::propagateWhileValidWithTrajectoryChecking(
    const ompl::base::State* state,
    const ompl::control::Control* control,
    unsigned int steps,
    std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker,
    const ompl::control::SpaceInformationPtr& si) const
{
    double stepSize = si->getPropagationStepSize();
    std::vector<ompl::base::State*> result;
    
    // Clear the result vector and add the starting state
    result.clear();
    result.push_back(si->cloneState(state));
    
    // If no steps requested, return immediately
    if (steps == 0) {
        return result;
    }
    
    // Check initial state validity using SAME method as planner
    if (!checkTrajectoryValidityAtStep(state, control, stepSize, validity_checker)) {
        return result; // Return only the initial state
    }
    
    ompl::base::State *current_state = si->cloneState(state);
    ompl::base::State *next_state = si->allocState();
    
    // Propagate step by step, checking validity at each step (SAME as planner)
    for (unsigned int i = 0; i < steps; ++i) {
        // Propagate one step forward
        si->getStatePropagator()->propagate(current_state, control, stepSize, next_state);
        
        // Check trajectory validity for this single step (SAME as planner)
        if (!checkTrajectoryValidityAtStep(current_state, control, stepSize, validity_checker)) {
            break; // Stop if trajectory becomes invalid
        }
        
        // Check if the resulting state is valid (SAME as planner)
        if (!si->isValid(next_state)) {
            break; // Stop if state becomes invalid
        }
        
        // Step is valid, add to result and continue
        result.push_back(si->cloneState(next_state));
        si->copyState(current_state, next_state);
    }
    
    // Clean up temporary states
    si->freeState(current_state);
    si->freeState(next_state);
    
    return result;
}

// Helper method that implements the SAME trajectory validity check as the planner
bool DiscreteContinuousExample::checkTrajectoryValidityAtStep(
    const ompl::base::State *current_state, 
    const ompl::control::Control *control, 
    double step_duration,
    std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker) const
{
    // Create vectors for single-step trajectory check (SAME as planner)
    std::vector<ompl::control::Control*> controls;
    std::vector<double> durations;
    
    // Add the current control and single step duration
    controls.push_back(const_cast<ompl::control::Control*>(control));
    durations.push_back(step_duration);
    
    // Check single-step trajectory validity (SAME as planner)
    return validity_checker->isValidTrajectory(current_state, controls, durations);
}

int main(int argc, char **argv)
{
    std::string config_file;
    
    if (argc > 1) {
        config_file = argv[1];
        std::cout << "Loading configuration from: " << config_file << std::endl;
    } else {
        std::cout << "No config file provided, using default values" << std::endl;
    }
    
    DiscreteContinuousExample example(config_file);
    
    std::cout << "=== STEP 1: Discrete Time Planning ===" << std::endl;
    example.planWithDiscreteTime();
    
    std::cout << "\n=== STEP 2: Continuous Time Validation ===" << std::endl;
    example.validateWithContinuousTime();
    
    std::cout << "\n=== Planning and Validation Complete ===" << std::endl;
    
    return 0;
}
