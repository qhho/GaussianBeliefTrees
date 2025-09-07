#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include "Planners/mod_rrt.hpp"
#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "Spaces/RNBeliefSpace.h"
#include "Spaces/RNBeliefSpace.h"
#include "StatePropagators/SimpleStatePropagator.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>

using namespace ompl;
using namespace ompl::base;
using namespace ompl::control;

// Custom goal region for belief states
class BeliefGoalRegion : public GoalRegion
{
public:
    BeliefGoalRegion(const ompl::base::SpaceInformationPtr &si, const State *goal_state, double threshold = 2.0)
        : GoalRegion(si), goal_state_(si->cloneState(goal_state)), threshold_(threshold)
    {
    }
    
    virtual ~BeliefGoalRegion()
    {
        si_->freeState(goal_state_);
    }
    
    virtual double distanceGoal(const State *state) const override
    {
        // Extract belief states
        auto state_belief = state->as<RNBeliefSpace::StateType>();
        auto goal_belief = goal_state_->as<RNBeliefSpace::StateType>();
        
        // Calculate Euclidean distance between means
        double dx = state_belief->getX() - goal_belief->getX();
        double dy = state_belief->getY() - goal_belief->getY();
        
        // std::cout << "State: " << state_belief->getX() << ", " << state_belief->getY() << std::endl;
        // std::cout << "Goal: " << goal_belief->getX() << ", " << goal_belief->getY() << std::endl;
        // std::cout << "Distance: " << std::sqrt(dx*dx + dy*dy) << std::endl;
        
        return std::sqrt(dx*dx + dy*dy);
    }

private:
    State *goal_state_;
    double threshold_;
};

int main()
{
    std::cout << "Testing Modified RRT with Barrier Trajectory Validity Checker" << std::endl;
    
    // Create state space (2D belief space)
    Eigen::MatrixXd sigma_init = 5.0 * Eigen::MatrixXd::Identity(2, 2);
    auto state_space = std::make_shared<RNBeliefSpace>(2, sigma_init);
    
    // Set bounds for the state space
    ompl::base::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0.0);
    bounds_se2.setHigh(50.0);
    state_space->setBounds(bounds_se2);
    
    // Create control space (3D: x_vel, y_vel, duration)
    auto control_space = std::make_shared<RealVectorControlSpace>(state_space, 3);
    
    // Set control bounds
    RealVectorBounds control_bounds(3);
    control_bounds.setLow(-1.0);  // Max velocity in any direction
    control_bounds.setHigh(1.0);
    control_space->setBounds(control_bounds);
    
    // Create space information
    auto si = std::make_shared<ompl::control::SpaceInformation>(state_space, control_space);
    
    // Create the barrier trajectory validity checker
    auto validity_checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    
    // Set system matrices for a simple 2D system
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd Q = 0.1 * Eigen::MatrixXd::Identity(2, 2);
    
    validity_checker->setSystemMatrices(A, B, K, G, Q);
    
    // Set half-space constraints to create a corridor
    std::vector<Eigen::VectorXd> a_list;
    std::vector<double> gamma_list;
    
    // Left boundary: x >= 0
    Eigen::VectorXd a1(2);
    a1 << -1.0, 0.0;
    a_list.push_back(a1);
    gamma_list.push_back(0.0);
    
    // Right boundary: x <= 50
    Eigen::VectorXd a2(2);
    a2 << 1.0, 0.0;
    a_list.push_back(a2);
    gamma_list.push_back(50.0);
    
    // Bottom boundary: y >= 0
    Eigen::VectorXd a3(2);
    a3 << 0.0, -1.0;
    a_list.push_back(a3);
    gamma_list.push_back(0.0);
    
    // Top boundary: y <= 50
    Eigen::VectorXd a4(2);
    a4 << 0.0, 1.0;
    a_list.push_back(a4);
    gamma_list.push_back(50.0);
    
    // Add obstacle constraint: avoid center region
    // Constraint: (x-25)^2 + (y-25)^2 >= 10^2 (avoid circle of radius 10 at center)
    // This is approximated with multiple half-space constraints
    for (int i = 0; i < 8; ++i) {
        double angle = 2.0 * M_PI * i / 8.0;
        Eigen::VectorXd a_obs(2);
        a_obs << -cos(angle), -sin(angle);
        a_list.push_back(a_obs);
        gamma_list.push_back(-25.0 * cos(angle) - 25.0 * sin(angle) + 2.0);
    }
    
    validity_checker->setHalfSpaceConstraints(a_list, gamma_list, 0.01); // 1% risk
    validity_checker->setTimeParameters(10, 0.1/10); // 10 steps, 0.1s each
    
    // Set the validity checker
    si->setStateValidityChecker(validity_checker);
    
    // Set state propagator with default parameters
    std::vector<std::vector<double>> measurement_regions = {{0.0, 100.0}, {0.0, 100.0}};

    double K_default = 0.8;

    si->setStatePropagator(std::make_shared<SimpleStatePropagator>(si, 0.1, 0.1, 0.2, K_default, measurement_regions));
    
    // Set propagation step size
    si->setPropagationStepSize(0.1);
    si->setMinControlDuration(1);
    si->setMaxControlDuration(5);
    
    // Setup the space information
    si->setup();
    
    // Create start and goal states
    auto start_state = state_space->allocState();
    auto start_belief = start_state->as<RNBeliefSpace::StateType>();
    start_belief->setX(5.0);
    start_belief->setY(5.0);
    start_belief->setSigma(0.5 * Eigen::MatrixXd::Identity(2, 2));
    
    auto goal_state = state_space->allocState();
    auto goal_belief = goal_state->as<RNBeliefSpace::StateType>();
    goal_belief->setX(40.0);
    goal_belief->setY(40.0);
    goal_belief->setSigma(0.5 * Eigen::MatrixXd::Identity(2, 2));
    
    // Create problem definition
    auto pdef = std::make_shared<ProblemDefinition>(si);
    pdef->addStartState(start_state);

    // Create custom goal region for belief states
    auto goal_region = std::make_shared<BeliefGoalRegion>(si, goal_state, 5.0);
    pdef->setGoal(goal_region);
    
    // Set optimization objective
    // auto objective = std::make_shared<PathLengthOptimizationObjective>(si);
    // pdef->setOptimizationObjective(objective);
    
    // Create the modified RRT planner
    auto planner = std::make_shared<control::mod_RRT>(si);
    planner->setProblemDefinition(pdef);
    
    // Set planner parameters
    planner->setGoalBias(0.1);
    planner->setSamplingBias(0.3);
    planner->setIntermediateStates(false);
    
    // Setup the planner
    planner->setup();
    
    std::cout << "Starting planning..." << std::endl;
    
    // Solve the problem
    auto start_time = std::chrono::high_resolution_clock::now();
    auto status = planner->solve(ompl::base::timedPlannerTerminationCondition(10.0)); // 10 second timeout
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (status == PlannerStatus::EXACT_SOLUTION || status == PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "Found solution!" << std::endl;
        if (status == PlannerStatus::APPROXIMATE_SOLUTION) {
            std::cout << "  (Approximate solution)" << std::endl;
        }
        
        // Get the solution path
        auto path = pdef->getSolutionPath();
        
        // For control problems, we need to cast to PathControl
        if (auto path_control = std::dynamic_pointer_cast<ompl::control::PathControl>(path)) {
            
            // Open files for writing states and controls
            std::ofstream states_file("solution_states.csv");
            std::ofstream controls_file("solution_controls.csv");
            
            // Write headers
            states_file << "x,y,sigma_trace,lambda_trace" << std::endl;
            controls_file << "ux,uy,duration,K" << std::endl;
            
            // Output each state and its associated control
            for (size_t i = 0; i < path_control->getStateCount(); ++i)
            {
                auto state = path_control->getState(i);
                auto belief = state->as<RNBeliefSpace::StateType>();
                
                // Write state to states file: x, y, sigma_trace, lambda_trace
                states_file << belief->getX() << "," << belief->getY() << "," << belief->getSigma().trace() << "," << belief->getLambda().trace() << std::endl;
                
                // Write control and duration to controls file if not the last state
                if (i < path_control->getStateCount() - 1) {
                    auto control = path_control->getControl(i);
                    auto real_control = control->as<ompl::control::RealVectorControlSpace::ControlType>();
                    double duration = path_control->getControlDuration(i);
                    
                    // Write control: ux, uy, duration, K
                    controls_file << real_control->values[0] << "," << real_control->values[1] << "," << duration << "," << K_default << std::endl;
                }
            }
            
            // Close files
            states_file.close();
            controls_file.close();
            
            std::cout << "Solution saved to:" << std::endl;
            std::cout << "  - solution_states.csv (states: x, y, sigma_trace, lambda_trace)" << std::endl;
            std::cout << "  - solution_controls.csv (controls: ux, uy, duration, K)" << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found." << std::endl;
        std::cout << "Status: " << status << std::endl;
    }
    
    std::cout << "Planning time: " << duration.count() << " ms" << std::endl;
    
    // Clean up
    state_space->freeState(start_state);
    state_space->freeState(goal_state);
    
    return 0;
}
