#include "barrier_rrt_main.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>

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
        
        return std::sqrt(dx*dx + dy*dy);
    }

private:
    State *goal_state_;
    double threshold_;
};

BarrierRRTMain::BarrierRRTMain()
{
    // Initialize with default values
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_configuration_.resize(2);
    goal_configuration_.resize(2);
    
    planning_bounds_x_[0] = 0.0;
    planning_bounds_x_[1] = 50.0;
    planning_bounds_y_[0] = 0.0;
    planning_bounds_y_[1] = 50.0;
    
    start_configuration_[0] = 5.0;
    start_configuration_[1] = 5.0;
    goal_configuration_[0] = 45.0;
    goal_configuration_[1] = 45.0;
    
    initial_covariance_ = 0.5 * Eigen::MatrixXd::Identity(2, 2);
    
    // Default system matrices
    A_ = Eigen::MatrixXd::Identity(2, 2);
    B_ = Eigen::MatrixXd::Identity(2, 2);
    K_ = Eigen::MatrixXd::Zero(2, 2);
    G_ = Eigen::MatrixXd::Identity(2, 2);
    Q_ = 0.1 * Eigen::MatrixXd::Identity(2, 2);
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
}

void BarrierRRTMain::loadConfig(const std::string& config_file)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    
    // Load system parameters
    dt_ = pt.get<double>("System.dt");
    Q_(0,0) = Q_(1,1) = pt.get<double>("System.Q");
    
    // Load planner parameters
    planning_time_ = pt.get<double>("Planner.planning_time");
    goal_bias_ = pt.get<double>("Planner.goal_bias");
    sampling_bias_ = pt.get<double>("Planner.sampling_bias");
    intermediate_states_ = pt.get<bool>("Planner.intermediate_states");
    
    // Load environment parameters
    std::string x_bounds = pt.get<std::string>("Environment.x_bounds");
    std::string y_bounds = pt.get<std::string>("Environment.y_bounds");
    std::string start_state = pt.get<std::string>("Planner.initial_state");
    std::string goal_state = pt.get<std::string>("Planner.goal");
    
    // Parse bounds and states (you'll need to implement parsing logic)
    // This is a simplified version - you may need more robust parsing
    
    // Load barrier parameters
    risk_threshold_ = pt.get<double>("Barrier.risk_threshold");
    time_steps_ = pt.get<int>("Barrier.time_steps");
    step_duration_ = pt.get<double>("Barrier.step_duration");
    
    // Load scene
    scene_name_ = pt.get<std::string>("Scene.scene");
}

void BarrierRRTMain::loadScene(const std::string& scene_file)
{
    YAML::Node config = YAML::LoadFile(scene_file);
    
    if (config["scene"]["obstacles"]) {
        for (const auto& obstacle : config["scene"]["obstacles"]) {
            if (obstacle["type"].as<std::string>() == "circle") {
                double center_x = obstacle["center_x"].as<double>();
                double center_y = obstacle["center_y"].as<double>();
                double radius = obstacle["radius"].as<double>();
                int num_constraints = obstacle["num_constraints"].as<int>();
                
                // Generate half-space constraints for circle
                for (int i = 0; i < num_constraints; ++i) {
                    double angle = 2.0 * M_PI * i / num_constraints;
                    Eigen::VectorXd a_obs(2);
                    a_obs << -cos(angle), -sin(angle);
                    obstacle_constraints_a_.push_back(a_obs);
                    obstacle_constraints_gamma_.push_back(-center_x * cos(angle) - center_y * sin(angle) + radius);
                }
            }
        }
    }
}

void BarrierRRTMain::setupBarrierConstraints()
{
    // Add boundary constraints
    std::vector<Eigen::VectorXd> a_list;
    std::vector<double> gamma_list;
    
    // Left boundary: x >= 0
    Eigen::VectorXd a1(2);
    a1 << -1.0, 0.0;
    a_list.push_back(a1);
    gamma_list.push_back(0.0);
    
    // Right boundary: x <= max_x
    Eigen::VectorXd a2(2);
    a2 << 1.0, 0.0;
    a_list.push_back(a2);
    gamma_list.push_back(planning_bounds_x_[1]);
    
    // Bottom boundary: y >= 0
    Eigen::VectorXd a3(2);
    a3 << 0.0, -1.0;
    a_list.push_back(a3);
    gamma_list.push_back(0.0);
    
    // Top boundary: y <= max_y
    Eigen::VectorXd a4(2);
    a4 << 0.0, 1.0;
    a_list.push_back(a4);
    gamma_list.push_back(planning_bounds_y_[1]);
    
    // Add obstacle constraints
    a_list.insert(a_list.end(), obstacle_constraints_a_.begin(), obstacle_constraints_a_.end());
    gamma_list.insert(gamma_list.end(), obstacle_constraints_gamma_.begin(), obstacle_constraints_gamma_.end());
}

void BarrierRRTMain::planWithBarrierRRT()
{
    std::cout << "Starting Barrier RRT Planning..." << std::endl;
    
    // Create state space (2D belief space)
    auto state_space = std::make_shared<RNBeliefSpace>(2, initial_covariance_);
    
    // Set bounds for the state space
    RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(planning_bounds_x_[0]);
    bounds_se2.setHigh(planning_bounds_x_[1]);
    bounds_se2.setLow(planning_bounds_y_[0]);
    bounds_se2.setHigh(planning_bounds_y_[1]);
    state_space->setBounds(bounds_se2);
    
    // Create control space (3D: x_vel, y_vel, duration)
    auto control_space = std::make_shared<RealVectorControlSpace>(state_space, 3);
    
    // Set control bounds
    RealVectorBounds control_bounds(3);
    control_bounds.setLow(-1.0);
    control_bounds.setHigh(1.0);
    control_space->setBounds(control_bounds);
    
    // Create space information - be explicit about control::SpaceInformation
    auto si = std::make_shared<ompl::control::SpaceInformation>(state_space, control_space);
    
    // Create the barrier trajectory validity checker
    auto validity_checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    
    // Set system matrices
    validity_checker->setSystemMatrices(A_, B_, K_, G_, Q_);
    
    // Setup barrier constraints
    setupBarrierConstraints();
    
    // Get constraints from setup
    std::vector<Eigen::VectorXd> a_list;
    std::vector<double> gamma_list;
    
    // Add boundary constraints
    Eigen::VectorXd a1(2), a2(2), a3(2), a4(2);
    a1 << -1.0, 0.0; a_list.push_back(a1); gamma_list.push_back(0.0);
    a2 << 1.0, 0.0; a_list.push_back(a2); gamma_list.push_back(planning_bounds_x_[1]);
    a3 << 0.0, -1.0; a_list.push_back(a3); gamma_list.push_back(0.0);
    a4 << 0.0, 1.0; a_list.push_back(a4); gamma_list.push_back(planning_bounds_y_[1]);
    
    // Add obstacle constraints
    a_list.insert(a_list.end(), obstacle_constraints_a_.begin(), obstacle_constraints_a_.end());
    gamma_list.insert(gamma_list.end(), obstacle_constraints_gamma_.begin(), obstacle_constraints_gamma_.end());
    
    validity_checker->setHalfSpaceConstraints(a_list, gamma_list, risk_threshold_);
    validity_checker->setTimeParameters(time_steps_, step_duration_);
    
    // Set the validity checker
    si->setStateValidityChecker(validity_checker);
    
    // Set state propagator
    std::vector<std::vector<double>> measurement_regions = {
        {planning_bounds_x_[0], planning_bounds_x_[1]}, 
        {planning_bounds_y_[0], planning_bounds_y_[1]}
    };
    si->setStatePropagator(std::make_shared<SimpleStatePropagator>(si, 0.1, 0.1, 0.2, 1.0, measurement_regions));
    
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
    auto goal_region = std::make_shared<BeliefGoalRegion>(si, goal_state, 10.0);
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
    
    if (status == PlannerStatus::EXACT_SOLUTION || status == PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "Found solution!" << std::endl;
        if (status == PlannerStatus::APPROXIMATE_SOLUTION) {
            std::cout << "  (Approximate solution)" << std::endl;
        }
        
        // Get the solution path
        auto path = pdef->getSolutionPath();
        
        if (auto path_control = std::dynamic_pointer_cast<PathControl>(path)) {
            saveSolutionPath(*path_control, state_space, "solution_barrier_rrt.csv");
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