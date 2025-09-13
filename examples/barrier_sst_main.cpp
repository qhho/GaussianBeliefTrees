#include "barrier_sst_main.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>


#include "barrier_sst_main.hpp"
#include <fstream>
#include <iostream>

// ============================
//  Scene Loading
// ============================
void BarrierSSTMain::loadScene(const std::string& scene_file)
{
    std::cout << "loadScene called with: '" << scene_file << "'" << std::endl;

    YAML::Node config = YAML::LoadFile(scene_file);

    if (config["scene"]["obstacles"]) {
        std::cout << "Found obstacles in scene file" << std::endl;

        for (const auto& obstacle : config["scene"]["obstacles"]) {
            std::vector<Eigen::VectorXd> obstacle_a_list;
            std::vector<double> obstacle_gamma_list;

            if (obstacle["type"]) {
                std::string obstacle_type = obstacle["type"].as<std::string>();
                if (obstacle_type == "circle") {
                    double center_x = obstacle["center_x"].as<double>();
                    double center_y = obstacle["center_y"].as<double>();
                    double radius = obstacle["radius"].as<double>();
                    int num_constraints = obstacle["num_constraints"].as<int>();

                    for (int i = 0; i < num_constraints; ++i) {
                        double angle = 2.0 * M_PI * i / num_constraints;
                        Eigen::VectorXd a_obs(2);
                        a_obs << cos(angle), sin(angle);
                        obstacle_a_list.push_back(a_obs);
                        obstacle_gamma_list.push_back(center_x * cos(angle) + center_y * sin(angle) + radius);
                    }
                }
            }
            else if (obstacle["fx"] && obstacle["tx"] && obstacle["fy"] && obstacle["ty"]) {
                double fx = obstacle["fx"].as<double>();
                double tx = obstacle["tx"].as<double>();
                double fy = obstacle["fy"].as<double>();
                double ty = obstacle["ty"].as<double>();

                Eigen::VectorXd a1(2); a1 << -1.0, 0.0;
                obstacle_a_list.push_back(a1);
                obstacle_gamma_list.push_back(-fx);

                Eigen::VectorXd a2(2); a2 << 1.0, 0.0;
                obstacle_a_list.push_back(a2);
                obstacle_gamma_list.push_back(tx);

                Eigen::VectorXd a3(2); a3 << 0.0, -1.0;
                obstacle_a_list.push_back(a3);
                obstacle_gamma_list.push_back(-fy);

                Eigen::VectorXd a4(2); a4 << 0.0, 1.0;
                obstacle_a_list.push_back(a4);
                obstacle_gamma_list.push_back(ty);
            }

            if (!obstacle_a_list.empty()) {
                obstacle_a_lists_.push_back(obstacle_a_list);
                obstacle_gamma_lists_.push_back(obstacle_gamma_list);
            }
        }
    }
}


// ============================
//  Barrier Constraints
// ============================
void BarrierSSTMain::setupBarrierConstraints()
{
    // Create the final multiple obstacles list
    std::vector<std::vector<Eigen::VectorXd>> final_obstacle_a_lists;
    std::vector<std::vector<double>> final_obstacle_gamma_lists;

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

    // Add all scene obstacles
    final_obstacle_a_lists.insert(final_obstacle_a_lists.end(),
                                  obstacle_a_lists_.begin(),
                                  obstacle_a_lists_.end());
    final_obstacle_gamma_lists.insert(final_obstacle_gamma_lists.end(),
                                      obstacle_gamma_lists_.begin(),
                                      obstacle_gamma_lists_.end());

    // Store final lists
    obstacle_a_lists_ = final_obstacle_a_lists;
    obstacle_gamma_lists_ = final_obstacle_gamma_lists;

    std::cout << "Setup complete: " << obstacle_a_lists_.size() << " obstacles total" << std::endl;
}



// ============================
//  Save Solution Path
// ============================
void BarrierSSTMain::saveSolutionPath(
    const ompl::control::PathControl& path_control,
    const ompl::base::StateSpacePtr& space,
    const std::string& filepath)
{
    std::ofstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "Could not open file to save path: " << filepath << std::endl;
        return;
    }

    // Write CSV header
    file << "x,y,sigma_trace,lambda_trace" << std::endl;

    for (size_t i = 0; i < path_control.getStateCount(); ++i)
    {
        auto belief = path_control.getState(i)->as<RNBeliefSpace::StateType>();

        file << belief->getX() << ","
             << belief->getY() << ","
             << belief->getSigma().trace() << ","
             << belief->getLambda().trace()
             << std::endl;
    }

    file.close();
    std::cout << "Saved solution path to: " << filepath << std::endl;
}


// ============================
//  Continuous-Time Validation
// ============================
bool BarrierSSTMain::validateWithContinuousTime(
    const ompl::control::PathControl& path_control,
    const ompl::control::SpaceInformationPtr& si,
    const ompl::base::StateSpacePtr& space)
{
    std::vector<ompl::base::State*> intermediate_states;
    std::vector<ompl::control::Control*> intermediate_controls;
    std::vector<double> intermediate_durations;

    generateIntermediateStates(path_control, si,
                               intermediate_states, intermediate_controls, intermediate_durations);

    std::cout << "Generated " << intermediate_states.size()
              << " intermediate states for continuous validation." << std::endl;

    bool all_valid = true;
    for (size_t i = 0; i < intermediate_states.size(); ++i)
    {
        if (!si->isValid(intermediate_states[i]))
        {
            std::cout << "Invalid intermediate state at step " << i << std::endl;
            all_valid = false;
            break;
        }
    }

    if (all_valid)
        saveIntermediateStates(intermediate_states, intermediate_controls, intermediate_durations,
                               "solution_continuous_valid.csv");
    else
        saveIntermediateStates(intermediate_states, intermediate_controls, intermediate_durations,
                               "solution_continuous_invalid.csv");

    for (auto* s : intermediate_states) space->freeState(s);
    for (auto* c : intermediate_controls) si->freeControl(c);

    return all_valid;
}


// ---------------- Goal region ----------------
class BeliefGoalRegion : public GoalRegion
{
public:
    BeliefGoalRegion(const ompl::control::SpaceInformationPtr &si, const State *goal_state, double threshold = 0.5)
        : GoalRegion(si), goal_state_(si->cloneState(goal_state)), threshold_(threshold) {}
    ~BeliefGoalRegion() { si_->freeState(goal_state_); }

    double distanceGoal(const State *state) const override
    {
        auto s = state->as<RNBeliefSpace::StateType>();
        auto g = goal_state_->as<RNBeliefSpace::StateType>();
        double dx = s->getX() - g->getX();
        double dy = s->getY() - g->getY();
        return (std::sqrt(dx*dx + dy*dy) < threshold_) ? 0.0 : std::sqrt(dx*dx + dy*dy);
    }

private:
    State *goal_state_;
    double threshold_;
};

// ============================
//  Generate Intermediate States
// ============================
void BarrierSSTMain::generateIntermediateStates(
    const ompl::control::PathControl& path_control,
    const ompl::control::SpaceInformationPtr& si,
    std::vector<ompl::base::State*>& intermediate_states,
    std::vector<ompl::control::Control*>& intermediate_controls,
    std::vector<double>& intermediate_durations)
{
    for (size_t i = 0; i < path_control.getControlCount(); ++i)
    {
        const auto* ctrl = path_control.getControl(i);
        double duration  = path_control.getControlDuration(i);
        const auto* start_state = path_control.getState(i);

        unsigned int steps = static_cast<unsigned int>(std::ceil(duration / step_duration_));
        double step_dur = duration / static_cast<double>(steps);

        auto states = propagateWhileValidWithTrajectoryChecking(
            start_state, ctrl, steps,
            std::make_shared<BarrierTrajectoryValidityChecker>(si), si);

        for (auto* s : states)
        {
            intermediate_states.push_back(si->cloneState(s));
            intermediate_controls.push_back(si->cloneControl(ctrl));
            intermediate_durations.push_back(step_dur);
        }
    }
}

// ============================
//  Propagate While Valid
// ============================
std::vector<ompl::base::State*> BarrierSSTMain::propagateWhileValidWithTrajectoryChecking(
    const ompl::base::State* state,
    const ompl::control::Control* control,
    unsigned int steps,
    std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker,
    const ompl::control::SpaceInformationPtr& si) const
{
    std::vector<ompl::base::State*> states;
    auto* current = si->cloneState(state);

    for (unsigned int i = 0; i < steps; ++i)
    {
        auto* next = si->allocState();
        si->getStatePropagator()->propagate(current, control, step_duration_, next);

        if (!checkTrajectoryValidityAtStep(current, control, step_duration_, validity_checker))
            break;

        states.push_back(si->cloneState(next));
        si->freeState(current);
        current = next;
    }

    si->freeState(current);
    return states;
}

// ============================
//  Check Trajectory Validity at Step
// ============================
bool BarrierSSTMain::checkTrajectoryValidityAtStep(
    const ompl::base::State* current_state,
    const ompl::control::Control* control,
    double step_duration,
    std::shared_ptr<BarrierTrajectoryValidityChecker> validity_checker) const
{
    return validity_checker->isValid(current_state);
}

// ============================
//  Save Intermediate States
// ============================
void BarrierSSTMain::saveIntermediateStates(
    const std::vector<ompl::base::State*>& states,
    const std::vector<ompl::control::Control*>& controls,
    const std::vector<double>& durations,
    const std::string& filepath)
{
    std::ofstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "Could not open file for writing intermediate states: " << filepath << std::endl;
        return;
    }

    for (size_t i = 0; i < states.size(); ++i)
    {
        const auto* st = states[i]->as<RNBeliefSpace::StateType>();
        file << st->getX() << "," << st->getY() << "," << durations[i] << std::endl;
    }

    file.close();
    std::cout << "Saved intermediate states to: " << filepath << std::endl;
}


// ---------------- Constructor ----------------
BarrierSSTMain::BarrierSSTMain(const std::string& config_file)
{
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_configuration_.resize(2);
    goal_configuration_.resize(2);

    if (!config_file.empty()) loadConfig(config_file);
    else {
        planning_bounds_x_ = {-10.0, 50.0};
        planning_bounds_y_ = {-10.0, 50.0};
        start_configuration_ = {5.0, 5.0};
        goal_configuration_ = {45.0, 45.0};
        initial_covariance_ = Eigen::MatrixXd::Identity(2, 2);

        A_ = B_ = G_ = Eigen::MatrixXd::Identity(2, 2);
        K_ = Eigen::MatrixXd::Zero(2, 2);
        Q_ = Eigen::MatrixXd::Identity(2, 2);
        R_ = 1.0; R_bad_ = 10.0; K_default_ = 0.8; dt_ = 0.1;

        planning_time_ = 10.0;
        selection_radius_ = 3.0;
        pruning_radius_   = 1.0;
        control_duration_ = {1, 5};

        risk_threshold_ = 0.01;
        time_steps_ = 50;
        step_duration_ = 0.002;

        scene_name_ = "scenes/2d_circle_approximation.yaml";
    }
}

// ---------------- Config Loader ----------------
void BarrierSSTMain::loadConfig(const std::string& config_file)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    auto parsePair = [](const std::string& s) {
        size_t pos = s.find(',');
        return std::pair<double,double>(std::stod(s.substr(0,pos)), std::stod(s.substr(pos+1)));
    };

    auto x_bounds = parsePair(pt.get<std::string>("Environment.x_bounds"));
    auto y_bounds = parsePair(pt.get<std::string>("Environment.y_bounds"));
    planning_bounds_x_ = {x_bounds.first, x_bounds.second};
    planning_bounds_y_ = {y_bounds.first, y_bounds.second};

    auto start = parsePair(pt.get<std::string>("Environment.start_configuration"));
    auto goal  = parsePair(pt.get<std::string>("Environment.goal_configuration"));
    start_configuration_ = {start.first, start.second};
    goal_configuration_  = {goal.first, goal.second};

    double init_cov = pt.get<double>("System.initial_covariance");
    initial_covariance_ = init_cov * Eigen::MatrixXd::Identity(2, 2);

    double Q_scalar = pt.get<double>("System.Q");
    R_ = pt.get<double>("System.R");
    R_bad_ = pt.get<double>("System.R_bad");
    K_default_ = pt.get<double>("System.K_default");
    dt_ = pt.get<double>("System.dt");

    A_ = B_ = G_ = Eigen::MatrixXd::Identity(2, 2);
    K_ = Eigen::MatrixXd::Zero(2, 2);
    Q_ = Q_scalar * Eigen::MatrixXd::Identity(2, 2);

    planning_time_ = pt.get<double>("Planner.planning_time");
    selection_radius_ = pt.get<double>("Planner.selection_radius");
    pruning_radius_   = pt.get<double>("Planner.pruning_radius");

    auto cd = parsePair(pt.get<std::string>("Planner.control_duration"));
    control_duration_ = {static_cast<int>(cd.first), static_cast<int>(cd.second)};

    risk_threshold_ = pt.get<double>("Barrier.risk_threshold");
    time_steps_     = pt.get<int>("Barrier.time_steps");
    step_duration_  = pt.get<double>("Barrier.step_duration");

    scene_name_ = pt.get<std::string>("Scene.scene");
    std::cout << "Loaded scene: " << scene_name_ << std::endl;
}

// ---------------- Planner ----------------
void BarrierSSTMain::planWithBarrierSST()
{
    std::cout << "Starting Barrier SST Planning...\n";

    auto state_space = std::make_shared<RNBeliefSpace>(2, initial_covariance_);
    RealVectorBounds bnds(2);
    bnds.setLow(0, planning_bounds_x_[0]);
    bnds.setHigh(0, planning_bounds_x_[1]);
    bnds.setLow(1, planning_bounds_y_[0]);
    bnds.setHigh(1, planning_bounds_y_[1]);
    state_space->setBounds(bnds);

    auto control_space = std::make_shared<RealVectorControlSpace>(state_space, 3);
    RealVectorBounds cb(3);
    cb.setLow(0,-1.0); cb.setHigh(0,1.0);
    cb.setLow(1,-1.0); cb.setHigh(1,1.0);
    cb.setLow(2,0.1);  cb.setHigh(2,0.9);
    control_space->setBounds(cb);

    auto si = std::make_shared<ompl::control::SpaceInformation>(state_space, control_space);

    auto checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    checker->setSystemMatrices(A_,B_,K_,G_,Q_);
    setupBarrierConstraints();
    checker->setMultipleObstacles(obstacle_a_lists_, obstacle_gamma_lists_, risk_threshold_);
    checker->setTimeParameters(time_steps_, step_duration_);
    si->setStateValidityChecker(checker);

    std::vector<std::vector<double>> meas = {
        {planning_bounds_x_[0], planning_bounds_x_[1]},
        {planning_bounds_y_[0], planning_bounds_y_[1]}
    };
    si->setStatePropagator(std::make_shared<SimpleStatePropagator>(
        si, Q_.trace()/2.0, R_, R_bad_, K_default_, meas));

    si->setPropagationStepSize(dt_);
    si->setMinControlDuration(control_duration_[0]);
    si->setMaxControlDuration(control_duration_[1]);
    si->setup();

    auto start_state = state_space->allocState();
    auto s = start_state->as<RNBeliefSpace::StateType>();
    s->setX(start_configuration_[0]);
    s->setY(start_configuration_[1]);
    s->setSigma(initial_covariance_);

    auto goal_state = state_space->allocState();
    auto g = goal_state->as<RNBeliefSpace::StateType>();
    g->setX(goal_configuration_[0]);
    g->setY(goal_configuration_[1]);
    g->setSigma(initial_covariance_);

    auto pdef = std::make_shared<ProblemDefinition>(si);
    pdef->addStartState(start_state);
    auto goal_region = std::make_shared<BeliefGoalRegion>(si, goal_state, 5.0);
    pdef->setGoal(goal_region);
    auto objective = std::make_shared<PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(objective);

    auto planner = std::make_shared<SST>(si);
    planner->setProblemDefinition(pdef);
    planner->setSelectionRadius(selection_radius_);
    planner->setPruningRadius(pruning_radius_);
    planner->setup();

    auto t0 = std::chrono::high_resolution_clock::now();
    auto status = planner->solve(timedPlannerTerminationCondition(planning_time_));
    auto t1 = std::chrono::high_resolution_clock::now();

    if (status == PlannerStatus::EXACT_SOLUTION || status == PlannerStatus::APPROXIMATE_SOLUTION) {
        std::cout << "Solution found\n";
        auto path = pdef->getSolutionPath();
        if (auto pc = std::dynamic_pointer_cast<PathControl>(path)) {
            double cost=0.0;
            for (size_t i=1;i<pc->getStateCount();++i)
                cost+=objective->motionCost(pc->getState(i-1),pc->getState(i)).value();
            std::ofstream f("solution_barrier_sst_cost.txt"); f<<cost<<"\n";
            saveSolutionPath(*pc, state_space, "solution_barrier_sst.csv");
            validateWithContinuousTime(*pc, si, state_space);
        }
    } else std::cout << "No solution\n";

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0);
    std::cout << "Planning time: " << dt.count() << " ms\n";

    state_space->freeState(start_state);
    state_space->freeState(goal_state);
}
