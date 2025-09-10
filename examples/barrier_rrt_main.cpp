#include "barrier_rrt_main.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>

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

BarrierRRTMain::BarrierRRTMain(const std::string& config_file)
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
        planning_bounds_x_[0] = -10.0;
        planning_bounds_x_[1] = 50.0;
        planning_bounds_y_[0] = -10.0;
        planning_bounds_y_[1] = 50.0;
        
        start_configuration_[0] = 5.0;
        start_configuration_[1] = 5.0;
        goal_configuration_[0] = 45.0;
        goal_configuration_[1] = 45.0;
        
        initial_covariance_ = 1.0 * Eigen::MatrixXd::Identity(2, 2);
        
        // Default system matrices
        A_ = Eigen::MatrixXd::Identity(2, 2);
        B_ = Eigen::MatrixXd::Identity(2, 2);
        K_ = Eigen::MatrixXd::Zero(2, 2);
        G_ = Eigen::MatrixXd::Identity(2, 2);
        Q_ = 1.0 * Eigen::MatrixXd::Identity(2, 2);  // Keep as matrix
        R_ = 1.0;
        R_bad_ = 10.0;
        K_default_ = 0.8;
        dt_ = 0.1;
        
        // Default planner parameters
        planning_time_ = 10.0;
        goal_bias_ = 0.1;
        sampling_bias_ = 0.3;
        intermediate_states_ = false;
        control_duration_ = {1, 5};
        
        // Default barrier parameters
        risk_threshold_ = 0.01;
        time_steps_ = 50;
        step_duration_ = 0.002;
        
        scene_name_ = "scenes/2d_circle_approximation.yaml";
        std::cout << "Using default scene name: '" << scene_name_ << "'" << std::endl;
    }
}

void BarrierRRTMain::loadConfig(const std::string& config_file)
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
    
    // Load system matrices
    double Q_scalar = pt.get<double>("System.Q");
    R_ = pt.get<double>("System.R");
    R_bad_ = pt.get<double>("System.R_bad");
    K_default_ = pt.get<double>("System.K_default");
    dt_ = pt.get<double>("System.dt");
    
    // Set system matrices
    A_ = Eigen::MatrixXd::Identity(2, 2);
    B_ = Eigen::MatrixXd::Identity(2, 2);
    K_ = Eigen::MatrixXd::Zero(2, 2);
    G_ = Eigen::MatrixXd::Identity(2, 2);
    Q_ = Q_scalar * Eigen::MatrixXd::Identity(2, 2);  // Convert scalar to matrix
    
    // Load planner parameters
    planning_time_ = pt.get<double>("Planner.planning_time");
    goal_bias_ = pt.get<double>("Planner.goal_bias");
    sampling_bias_ = pt.get<double>("Planner.sampling_bias");
    intermediate_states_ = pt.get<bool>("Planner.intermediate_states");
    
    // Parse control duration (format: "min,max")
    std::string control_dur = pt.get<std::string>("Planner.control_duration");
    comma_pos = control_dur.find(',');
    control_duration_[0] = std::stod(control_dur.substr(0, comma_pos));
    control_duration_[1] = std::stod(control_dur.substr(comma_pos + 1));
    
    // Load barrier parameters
    risk_threshold_ = pt.get<double>("Barrier.risk_threshold");
    time_steps_ = pt.get<int>("Barrier.time_steps");
    step_duration_ = pt.get<double>("Barrier.step_duration");
    
    // Load scene
    scene_name_ = pt.get<std::string>("Scene.scene");
    std::cout << "Loaded scene name from config: '" << scene_name_ << "'" << std::endl;
}

void BarrierRRTMain::loadScene(const std::string& scene_file)
{
    std::cout << "loadScene called with: '" << scene_file << "'" << std::endl;
    
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

void BarrierRRTMain::setupBarrierConstraints()
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

void BarrierRRTMain::planWithBarrierRRT()
{
    std::cout << "Starting Barrier RRT Planning..." << std::endl;
    std::cout << "Scene name from config: '" << scene_name_ << "'" << std::endl;
    
    // Note: Scene should be loaded before calling this method
    // (e.g., via loadScene() or loadConfig() which calls loadScene())
    
    // Create state space (2D belief space)
    auto state_space = std::make_shared<RNBeliefSpace>(2, initial_covariance_);
    
    // Set bounds for the state space
    RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, planning_bounds_x_[0]);
    bounds_se2.setHigh(0, planning_bounds_x_[1]);
    bounds_se2.setLow(1, planning_bounds_y_[0]);
    bounds_se2.setHigh(1, planning_bounds_y_[1]);
    state_space->setBounds(bounds_se2);
    
    // Create control space (3D: x_vel, y_vel, duration)
    auto control_space = std::make_shared<RealVectorControlSpace>(state_space, 3);
    
    // Set control bounds
    RealVectorBounds control_bounds(3);
    control_bounds.setLow(0, -1.0);
    control_bounds.setHigh(0, 1.0);
    control_bounds.setLow(1, -1.0);
    control_bounds.setHigh(1, 1.0);
    control_bounds.setLow(2, 0.1);
    control_bounds.setHigh(2, 0.9);
    control_space->setBounds(control_bounds);
    
    // Create space information
    auto si = std::make_shared<ompl::control::SpaceInformation>(state_space, control_space);
    
    // Create the barrier trajectory validity checker
    auto validity_checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    
    // Set system matrices
    validity_checker->setSystemMatrices(A_, B_, K_, G_, Q_);  // Now Q_ is a matrix
    
    // Setup barrier constraints
    setupBarrierConstraints();
    
    // Set multiple obstacles constraints (each obstacle checked separately)
    validity_checker->setMultipleObstacles(obstacle_a_lists_, obstacle_gamma_lists_, risk_threshold_);
    validity_checker->setTimeParameters(time_steps_, step_duration_);
    
    // Set the validity checker
    si->setStateValidityChecker(validity_checker);
    
    // Set state propagator with config values
    std::vector<std::vector<double>> measurement_regions = {
        {planning_bounds_x_[0], planning_bounds_x_[1]}, 
        {planning_bounds_y_[0], planning_bounds_y_[1]}
    };
    si->setStatePropagator(std::make_shared<SimpleStatePropagator>(si, Q_.trace()/2.0, R_, R_bad_, K_default_, measurement_regions));
    
    // Set propagation parameters
    si->setPropagationStepSize(dt_);
    si->setMinControlDuration(control_duration_[0]);
    si->setMaxControlDuration(control_duration_[1]);
    
    // Setup the space information
    si->setup();
    
    // Create start and goal states
    auto start_state = state_space->allocState();
    auto start_belief = start_state->as<RNBeliefSpace::StateType>();
    start_belief->setX(start_configuration_[0]);
    start_belief->setY(start_configuration_[1]);
    start_belief->setSigma(initial_covariance_);
    
    auto goal_state = state_space->allocState();
    auto goal_belief = goal_state->as<RNBeliefSpace::StateType>();
    goal_belief->setX(goal_configuration_[0]);
    goal_belief->setY(goal_configuration_[1]);
    goal_belief->setSigma(initial_covariance_);
    
    // Create problem definition
    auto pdef = std::make_shared<ProblemDefinition>(si);
    pdef->addStartState(start_state);
    
    // Create custom goal region
    auto goal_region = std::make_shared<BeliefGoalRegion>(si, goal_state, 5.0);
    pdef->setGoal(goal_region);
    
    // Set optimization objective
    auto objective = std::make_shared<PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(objective);
    
    // Create the modified RRT planner
    auto planner = std::make_shared<control::mod_RRT>(si);
    planner->setProblemDefinition(pdef);
    
    // Set planner parameters
    planner->setGoalBias(goal_bias_);
    planner->setSamplingBias(sampling_bias_);
    planner->setIntermediateStates(intermediate_states_);
    
    // Setup the planner
    planner->setup();
    
    std::cout << "Starting planning..." << std::endl;
    
    // Solve the problem
    auto start_time = std::chrono::high_resolution_clock::now();
    auto status = planner->solve(timedPlannerTerminationCondition(planning_time_));
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (status == PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution!" << std::endl;
        if (status == PlannerStatus::APPROXIMATE_SOLUTION) {
            std::cout << "  (Approximate solution)" << std::endl;
        }
        
        // Get the solution path
        auto path = pdef->getSolutionPath();
        
        if (auto path_control = std::dynamic_pointer_cast<PathControl>(path)) {
            // Compute and report path cost using the objective (feasibility planner won't optimize it)
            double total_cost = 0.0;
            for (size_t i = 1; i < path_control->getStateCount(); ++i) {
                auto c = objective->motionCost(path_control->getState(i-1), path_control->getState(i));
                total_cost += c.value();
            }
            std::cout << "Path cost (objective): " << total_cost << std::endl;
            {
                std::ofstream cost_file("solution_barrier_rrt_cost.txt");
                cost_file << total_cost << std::endl;
            }

            saveSolutionPath(*path_control, state_space, "solution_barrier_rrt.csv");

            // Validate with continuous-time barrier checker
            std::cout << "\n=== Continuous Time Validation ===" << std::endl;
            validateWithContinuousTime(*path_control, si, state_space);
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
}

void BarrierRRTMain::saveSolutionPath(const PathControl& path_control, 
                                     const StateSpacePtr& space, 
                                     const std::string& filepath)
{
    std::ofstream output_file(filepath);
    output_file << "x,y,sigma_trace,lambda_trace" << std::endl;
    
    for (size_t i = 0; i < path_control.getStateCount(); ++i) {
        auto state = path_control.getState(i);
        auto belief = state->as<RNBeliefSpace::StateType>();
        
        output_file << belief->getX() << "," 
                   << belief->getY() << "," 
                   << belief->getSigma().trace() << "," 
                   << belief->getLambda().trace() << std::endl;
    }
    
    output_file.close();
    std::cout << "Solution saved to: " << filepath << std::endl;
} 

// FIXED Continuous-time validation methods
void BarrierRRTMain::generateIntermediateStates(const PathControl& path_control, 
                                               const ompl::control::SpaceInformationPtr& si,
                                               std::vector<ompl::base::State*>& intermediate_states,
                                               std::vector<ompl::control::Control*>& intermediate_controls,
                                               std::vector<double>& intermediate_durations)
{
    std::cout << "Generating intermediate states using SAME validation as planner..." << std::endl;
    
    // Get the states and controls from the path
    std::vector<ompl::base::State*> path_states = const_cast<PathControl&>(path_control).getStates();
    std::vector<ompl::control::Control*> path_controls = const_cast<PathControl&>(path_control).getControls();
    
    // Clear output vectors
    intermediate_states.clear();
    intermediate_controls.clear();
    intermediate_durations.clear();
    
    // Create the barrier trajectory validity checker (SAME as planner)
    auto validity_checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    validity_checker->setSystemMatrices(A_, B_, K_, G_, Q_);
    validity_checker->setMultipleObstacles(obstacle_a_lists_, obstacle_gamma_lists_, risk_threshold_);
    validity_checker->setTimeParameters(time_steps_, step_duration_);
    
    // For each control segment, use the SAME validation approach as the planner
    for (size_t i = 0; i < path_controls.size(); ++i) {
        double duration = path_control.getControlDuration(i);
        int num_steps = static_cast<int>(duration / dt_);
        
        // std::cout << "Control " << i << ": duration = " << duration 
                //   << ", generating " << num_steps << " intermediate states" << std::endl;
        
        // Use the SAME validation approach as propagateWhileValidWithTrajectoryChecking
        std::vector<ompl::base::State*> pstates = propagateWhileValidWithTrajectoryChecking(
            path_states[i], path_controls[i], num_steps, validity_checker, si);
        
        // std::cout << "  Actual steps propagated: " << pstates.size() << std::endl;
        
        // Add all propagated states to our intermediate states
        for (size_t j = 0; j < pstates.size(); ++j) {
            intermediate_states.push_back(pstates[j]);
            intermediate_controls.push_back(path_controls[i]);
            intermediate_durations.push_back(dt_);
        }
    }
    
    // std::cout << "Generated " << intermediate_states.size() << " intermediate states using SAME validation as planner" << std::endl;
}

// Helper method that implements the SAME validation logic as the planner
std::vector<ompl::base::State*> BarrierRRTMain::propagateWhileValidWithTrajectoryChecking(
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
bool BarrierRRTMain::checkTrajectoryValidityAtStep(
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

bool BarrierRRTMain::validateWithContinuousTime(const PathControl& path_control,
                                               const ompl::control::SpaceInformationPtr& si,
                                               const StateSpacePtr& space)
{
    std::cout << "Starting continuous time validation using SAME approach as planner..." << std::endl;
    
    // Generate intermediate states using the SAME validation approach as the planner
    std::vector<ompl::base::State*> intermediate_states;
    std::vector<ompl::control::Control*> intermediate_controls;
    std::vector<double> intermediate_durations;
    
    generateIntermediateStates(path_control, si, intermediate_states, intermediate_controls, intermediate_durations);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Check if the trajectory was fully valid by comparing expected vs actual states
    // If any step failed validation, we'll have fewer states than expected
    std::vector<ompl::base::State*> path_states = const_cast<PathControl&>(path_control).getStates();
    std::vector<ompl::control::Control*> path_controls = const_cast<PathControl&>(path_control).getControls();
    
    // Calculate expected number of intermediate states
    int expected_states = 0;
    for (size_t i = 0; i < path_controls.size(); ++i) {
        double duration = path_control.getControlDuration(i);
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
    
    return is_fully_valid;
}

void BarrierRRTMain::saveIntermediateStates(const std::vector<ompl::base::State*>& states, 
                                          const std::vector<ompl::control::Control*>& controls,
                                          const std::vector<double>& durations,
                                          const std::string& filepath)
{
    std::ofstream output_file(filepath);
    output_file << "x,y,sigma_trace,lambda_trace,control_x,control_y,control_duration" << std::endl;
    
    for (size_t i = 0; i < states.size(); ++i) {
        auto state = states[i];
        auto belief = state->as<RNBeliefSpace::StateType>();
        
        // Get control information
        auto control = controls[i];
        auto control_values = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        double duration = durations[i];
        
        output_file << belief->getX() << "," 
                   << belief->getY() << "," 
                   << belief->getSigma().trace() << "," 
                   << belief->getLambda().trace() << ","
                   << control_values[0] << ","
                   << control_values[1] << ","
                   << duration << std::endl;
    }
    
    output_file.close();
    std::cout << "Intermediate states saved to: " << filepath << std::endl;
}
