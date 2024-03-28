/*

 */

#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <fstream>
#include "gbt.hpp"
#include <filesystem>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

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
{   
    
    scene_ = Scene(scene_config);
    system_ = System(system_config);

    // TODO: Should this be hardcoded to a 2x2? Will need to change when we make R^n.
    initial_covariance_ = 1.0*Eigen::MatrixXd::Identity(2, 2);
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

void GaussianBeliefTrees::solve(ob::PlannerPtr planner)
{
    //=======================================================================
    // Attempt to solve the problem
    //=======================================================================
    if(simple_setup_->haveExactSolutionPath())
    {
        oc::PathControl path_control = simple_setup_->getSolutionPath();
        path_control.print(std::cout);
        std::ofstream outputfile;

        path_control.printAsMatrix(outputfile);

        og::PathGeometric path = path_control.asGeometric();
        std::cout << "path with length " << path.length() << "has been found with simple_setup" << std::endl;

        std::vector<ob::State*> path_control_states;
        path_control_states = path_control.getStates();
        std::vector<oc::Control*> path_control_controls;
        path_control_controls = path_control.getControls();

        ob::StateSpacePtr space = simple_setup_->getStateSpace();
        oc::ControlSpacePtr control_space = simple_setup_->getControlSpace();

        path_states_.clear();
        path_states_.reserve(path_control_states.size()-1);

        path_controls_.clear();
        path_controls_.reserve(path_control_controls.size());

        for (int i = path_control_controls.size()-1; i >= 0; i--)
        {
            oc::Control *c = control_space->allocControl();
            control_space->copyControl(c, path_control_controls[i]);
            path_controls_.push_back(c);
        }
        for (size_t i = path_control_states.size()-1; i > 0; i--)
        {
            ob::State *s = space->allocState();
            space->copyState(s, path_control_states[i]);
            path_states_.push_back(s);

            double x_pose = s->as<R2BeliefSpace::StateType>()->getX();
            double y_pose = s->as<R2BeliefSpace::StateType>()->getY();
            Mat x_cov = s->as<R2BeliefSpace::StateType>()->getCovariance();
            std::cout << "x_pose: " << x_pose << std::endl;
            std::cout << "y_pose: " << y_pose << std::endl;
            std::cout << "cov: " << x_cov << std::endl;
        }

        oc::SpaceInformationPtr si = simple_setup_->getSpaceInformation();
        double total_duration=0;
        for(int i=0 ;i<path_control_controls.size(); i++)
        {
            std::cout << "path_control.getControlDuration(): " << path_control.getControlDuration(i) << std::endl;
            total_duration = total_duration + path_control.getControlDuration(i);
        }
        std::cout << "total_duration: " << total_duration << std::endl;
        std::cout << "path_control_states.size(): " << path_control_states.size() << std::endl;
        std::cout << "path_control_controls.size(): " << path_control_controls.size() << std::endl;

        //=======================================================================
        // Clear previous solution path
        //=======================================================================
        std::cout << "clearing" << std::endl;
        planner->clear();

        exit(0);
    }
    else
    {
        std::cout << "path has not been found" << std::endl;

        oc::PathControl path_control = simple_setup_->getSolutionPath();
        path_control.print(std::cout);

        og::PathGeometric path = path_control.asGeometric();
        std::cout << "path with length " << path.length() << "has been found with simple_setup" << std::endl;

        std::vector<ob::State*> path_control_states;
        path_control_states = path_control.getStates();
        std::vector<oc::Control*> path_control_controls;
        path_control_controls = path_control.getControls();

        ob::StateSpacePtr space = simple_setup_->getStateSpace();
        oc::ControlSpacePtr control_space = simple_setup_->getControlSpace();

        path_states_.clear();
        path_states_.reserve(path_control_states.size()-1);

        path_controls_.clear();
        path_controls_.reserve(path_control_controls.size());
        exit(0);
    }
}

int main(int argc, char **argv)
{   
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path scene_config = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::filesystem::path system_config = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    GaussianBeliefTrees GaussianBeliefTrees(scene_config, system_config);
    GaussianBeliefTrees.planWithSimpleSetup();
    return 0;
}
