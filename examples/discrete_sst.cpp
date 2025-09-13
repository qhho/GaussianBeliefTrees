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
#include <ompl/control/planners/sst/SST.h>       // ✅ only SST
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// headers
#include "ValidityCheckers/state_validity_checker_pcc_blackmore.hpp"
#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "StatePropagators/SimpleStatePropagator.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RNBeliefSpace.h"
#include "OptimizationObjectives/state_cost_objective.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;

// ===========================
// Custom goal region
// ===========================
class BeliefGoalRegion : public ob::GoalRegion
{
public:
    BeliefGoalRegion(const oc::SpaceInformationPtr &si, const ob::State *goal_state, double threshold = 0.5)
        : ob::GoalRegion(si), goal_state_(si->cloneState(goal_state)), threshold_(threshold) {}

    ~BeliefGoalRegion() override { si_->freeState(goal_state_); }

    double distanceGoal(const ob::State *state) const override
    {
        auto state_belief = state->as<RNBeliefSpace::StateType>();
        auto goal_belief  = goal_state_->as<RNBeliefSpace::StateType>();

        double dx = state_belief->getX() - goal_belief->getX();
        double dy = state_belief->getY() - goal_belief->getY();

        if (std::sqrt(dx*dx + dy*dy) < threshold_)
            return 0.0;

        return std::sqrt(dx*dx + dy*dy);
    }

private:
    ob::State *goal_state_;
    double threshold_;
};

// ===========================
// Main Example Class
// ===========================
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

private:
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
    Eigen::MatrixXd A_, B_, K_, G_, Q_;
    double dt_;
    double risk_threshold_;
    int time_steps_;
    double step_duration_;
    std::vector<std::vector<Eigen::VectorXd>> obstacle_a_lists_;
    std::vector<std::vector<double>> obstacle_gamma_lists_;
    std::shared_ptr<oc::PathControl> solution_path_;
    bool solution_found_;
};

// ===========================
// Constructor (defaults only shown)
// ===========================
DiscreteContinuousExample::DiscreteContinuousExample(const std::string& config_file)
    : solution_path_(nullptr), solution_found_(false)
{
    planning_bounds_x_ = {0.0, 100.0};
    planning_bounds_y_ = {0.0, 100.0};
    start_configuration_ = {10.0, 10.0};
    goal_configuration_  = {90.0, 90.0};
    initial_covariance_  = 1.0 * Eigen::MatrixXd::Identity(2, 2);
    scene_name_ = "scene3";
    solving_time_ = 60.0;
    Q_noise_ = 0.2;
    R_noise_ = 0.1;
    R_bad_ = 5.0;
    K_default_ = 0.9;
    A_ = Eigen::MatrixXd::Identity(2, 2);
    B_ = Eigen::MatrixXd::Identity(2, 2);
    K_ = Eigen::MatrixXd::Zero(2, 2);
    G_ = Eigen::MatrixXd::Identity(2, 2);
    Q_ = Q_noise_ * Eigen::MatrixXd::Identity(2, 2);
    dt_ = 0.1;
    risk_threshold_ = 0.01;
    time_steps_ = 50;
    step_duration_ = 0.002;
}

// ===========================
// The SST planner
// ===========================
void DiscreteContinuousExample::planWithDiscreteTime()
{
    ob::StateSpacePtr space(new RNBeliefSpace(2, initial_covariance_));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, planning_bounds_x_[0]);
    bounds.setHigh(0, planning_bounds_x_[1]);
    bounds.setLow(1, planning_bounds_y_[0]);
    bounds.setHigh(1, planning_bounds_y_[1]);
    space->as<RNBeliefSpace>()->setBounds(bounds);

    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(0, -10.0); cbounds.setHigh(0, 10.0);
    cbounds.setLow(1, -10.0); cbounds.setHigh(1, 10.0);
    cbounds.setLow(2, 0.0);   cbounds.setHigh(2, 0.6);
    cspace->setBounds(cbounds);

    oc::SpaceInformationPtr si(new oc::SpaceInformation(space, cspace));
    si->setMinMaxControlDuration(1, 5);
    si->setPropagationStepSize(0.1);

    // ✅ SST setup
    double goal_bias = 0.05;
    double selection_radius = 0.5;
    double pruning_radius   = 0.2;

    ob::PlannerPtr planner = std::make_shared<oc::SST>(si);
    auto sst = planner->as<oc::SST>();
    sst->setGoalBias(goal_bias);
    sst->setSelectionRadius(selection_radius);
    sst->setPruningRadius(pruning_radius);

    // Start & goal
    ob::ScopedState<> start(space);
    start[0] = start_configuration_[0];
    start[1] = start_configuration_[1];

    ob::ScopedState<> goal(space);
    goal[0] = goal_configuration_[0];
    goal[1] = goal_configuration_[1];

    si->setStatePropagator(std::make_shared<SimpleStatePropagator>(si, Q_noise_, R_noise_, R_bad_, K_default_,
                                                                   std::vector<std::vector<double>>{
                                                                       {planning_bounds_x_[0], planning_bounds_x_[1]},
                                                                       {planning_bounds_y_[0], planning_bounds_y_[1]}
                                                                   }));

    si->setStateValidityChecker(std::make_shared<StateValidityCheckerPCCBlackmore>(scene_name_, si, 0.99, 0));
    si->setup();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<BeliefGoalRegion>(si, goal.get(), 5.0));
    pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    std::cout << "Planning with SST..." << std::endl;
    ob::PlannerStatus status = planner->solve(solving_time_);
    if (status)
        std::cout << "✅ Solution found" << std::endl;
    else
        std::cout << "❌ No solution" << std::endl;
}

// ===========================
// main()
// ===========================
int main(int argc, char **argv)
{
    std::string config_file;
    if (argc > 1) config_file = argv[1];
    DiscreteContinuousExample example(config_file);
    example.planWithDiscreteTime();
    return 0;
}