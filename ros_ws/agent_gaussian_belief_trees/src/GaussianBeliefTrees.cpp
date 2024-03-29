#include "agent_gaussian_belief_trees/GaussianBeliefTrees.h"
#include "agent_helpers/Scene.h"
#include "agent_helpers/System.h"
#include <chrono>
#include <functional>
#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <fstream>
#include <filesystem>
#include "Spaces/R2BeliefSpace.h"
#include "Planners/SSBT.hpp"
#include "OptimizationObjectives/state_cost_objective.hpp"
#include "StatePropagators/SimpleStatePropagator.h"
#include "ValidityCheckers/StateValidityCheckerBlackmore.h"

using namespace std::chrono_literals;
typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;

ob::StateSpacePtr constructStateSpace(void)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new R2BeliefSpace(2.0));
    return state_space;
}

ob::StateSpacePtr constructRealVectorStateSpace(void)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
    return state_space;
}

class MyGoalRegion : public ompl::base::GoalRegion
{
    public:
        MyGoalRegion(const oc::SpaceInformationPtr &si) : ompl::base::GoalRegion(si)
        {
            setThreshold(16.0);
        }
    
        virtual double distanceGoal(const State *st) const
            {

            double dx = st->as<R2BeliefSpace::StateType>()->getX() - 90.0;
            double dy = st->as<R2BeliefSpace::StateType>()->getY() - 90.0;

            return (dx*dx + dy*dy);
        }
};

GaussianBeliefTrees::GaussianBeliefTrees(const std::string& scene_config, const std::string& system_config)
: Node("gbt_publisher")
{
    scene_ = Scene(scene_config);
    system_ = System(system_config);
    initial_covariance_ = 1.0*Eigen::MatrixXd::Identity(2, 2);
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&GaussianBeliefTrees::timer_callback, this));
    offboardSetpointCounter_ = 0;
}

void GaussianBeliefTrees::timer_callback()
{       
    std::cout<<offboardSetpointCounter_<<std::endl;

    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    for (const auto& obstacle : scene_.obstacles_limits_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE; 
        marker.action = visualization_msgs::msg::Marker::ADD;
        auto& min_point = obstacle.first;
        auto& max_point = obstacle.second;
        marker.pose.position.x = (min_point[0] + max_point[0]) / 2.0f;
        marker.pose.position.y = (min_point[1] + max_point[1]) / 2.0f;
        marker.pose.position.z = (min_point[2] + max_point[2]) / 2.0f;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::abs(max_point[0] - min_point[0]);
        marker.scale.y = std::abs(max_point[1] - min_point[1]);
        marker.scale.z = std::abs(max_point[2] - min_point[2]);
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
    }

    publisher_->publish(marker_array);

    if (offboardSetpointCounter_ < 1) {
        offboardSetpointCounter_++;
    }
}

void GaussianBeliefTrees::SaveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string stringpath)
{   
    std::ofstream outputsolution;

    outputsolution.open(stringpath, std::ios::out | std::ios::trunc);
    outputsolution << "x,y,cov" << std::endl;
    
    std::vector<ob::State*> path_control_states;
    path_control_states = path_control.getStates();

    for (size_t i = path_control_states.size()-1; i > 0; i--)
    {
        ob::State *s = space->allocState();
        space->copyState(s, path_control_states[i]);
        path_states_.push_back(s);

        double x_pose = s->as<R2BeliefSpace::StateType>()->getX();
        double y_pose = s->as<R2BeliefSpace::StateType>()->getY();
        Mat cov = s->as<R2BeliefSpace::StateType>()->getCovariance();

        outputsolution << x_pose << ","  << y_pose << "," <<  cov.trace() << std::endl;

    }
    outputsolution << "10.0,10.0,10.0" << std::endl;

    outputsolution.close();

}

void GaussianBeliefTrees::planWithSimpleSetup()
{
    ob::StateSpacePtr space(constructStateSpace()); //space should probably be a class attribute
    ob::RealVectorBounds bounds_se_n(scene_.scene_bounds_.size());
    for(int i = 0; i < scene_.scene_bounds_.size(); i++)
    {
        bounds_se_n.setLow(i, scene_.scene_bounds_[i].first);
        bounds_se_n.setHigh(i, scene_.scene_bounds_[i].second);
    }
    space->as<R2BeliefSpace>()->setBounds(bounds_se_n); //TODO: currently setup for se2 how to make se^n?
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, system_.control_bounds_.size()));
    ob::RealVectorBounds bounds(system_.control_bounds_.size());
    for(int i = 0; i < system_.control_bounds_.size(); i++)
    {
        bounds.setLow(i, system_.control_bounds_[i].first);
        bounds.setHigh(i, system_.control_bounds_[i].second);
    }
    cspace->setBounds(bounds);
    oc::SpaceInformationPtr si(new oc::SpaceInformation(space, cspace));
    si->setMinMaxControlDuration(system_.control_duration_.first, system_.control_duration_.second);
    si->setPropagationStepSize(system_.propagation_size_);

    ob::PlannerPtr planner;
    planner = ob::PlannerPtr(new oc::SSBT(si));
    planner->as<oc::SSBT>()->setGoalBias(system_.goal_bias_);
    planner->as<oc::SSBT>()->setSelectionRadius(system_.selection_radius_);
    planner->as<oc::SSBT>()->setPruningRadius(system_.pruning_radius_);
    planner->as<oc::SSBT>()->setSamplingBias(system_.sampling_bias_);
    planner->as<oc::SSBT>()->setDistanceFunction(1);
    
    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    // create a start state
    ob::ScopedState<> start(space);
    for(int i = 0; i < system_.starting_configuration_.size(); i++)
    {
        start[i] = system_.starting_configuration_[i];
    }
    // create a goal state
    ob::ScopedState<> goal(space);
    for(int i = 0; i < system_.goal_configuration_.size(); i++)
    {
        goal[i] = system_.goal_configuration_[i];
    }

    si->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si, system_.Q_,system_.R_)));

    ob::StateValidityCheckerPtr om_stat_val_check;
    // om_stat_val_check = ob::StateValidityCheckerPtr(new Scenario2ValidityChecker(si));
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerBlackmore(scene_, si, system_.p_safe_));
    // simple_setup_->setStateValidityChecker(om_stat_val_check);
    si->setStateValidityChecker(om_stat_val_check);

    si->setup();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<MyGoalRegion>(si));
    pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));
    pdef->getOptimizationObjective()->setCostThreshold(ob::Cost(system_.cost_threshold_));

    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->solve(system_.planning_time_);

    ob::PlannerData planner_data(si);
    planner->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    std::ofstream outputfile;
    outputfile.open("fixedK_03_tree_60sec.csv", std::ios::out | std::ios::trunc);
    outputfile << "to,from,x,y,cov,cost" << std::endl;
    
    for(unsigned int i = 1; i < planner_data.numVertices(); ++i) {

        double x = planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getX();
        double y = planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getY();
        Mat cov = planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getCovariance();
        if (planner_data.getIncomingEdges(i, edgeList) > 0){
            outputfile << i << "," << edgeList[0] << "," << x << "," << y << "," << cov.trace() << "," << planner_data.getVertex(i).getState()->as<R2BeliefSpace::StateType>()->getCost() << std::endl; 
        }
    }

    outputfile.close();

    if (planner->getProblemDefinition()->hasExactSolution()){
    const ompl::base::PathPtr &path_5sec = planner->getProblemDefinition()->getSolutionPath(); 
    oc::PathControl path_control = static_cast<oc::PathControl&>(*path_5sec);
    this->SaveSolutionPath(path_control, space, "solution_60sec_fixedK_03.csv");
    }
    else{
        std::cout << "No solution for 10 secs" << std::endl;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path sceneConfig = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::filesystem::path systemConfig = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    auto node = std::make_shared<GaussianBeliefTrees>(sceneConfig, systemConfig);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}