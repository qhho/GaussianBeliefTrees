/*

 */

#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <yaml-cpp/yaml.h>

// OMPL
// #include <ompl/control/SpaceInformation.h>
// #include <ompl/control/planners/PlannerIncludes.h>
// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/spaces/SE2StateSpace.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/planners/rrt/RRT.h>
// #include <ompl/geometric/planners/prm/PRMstar.h>
// #include <ompl/geometric/SimpleSetup.h>
// #include <ompl/config.h>

// headers
// #include "ValidityCheckers/Scenario1ValidityChecker.hpp" // simple validity checker
// #include "ValidityCheckers/Scenario2ValidityChecker.hpp" // simple validity checker
// #include "ValidityCheckers/Scenario3ValidityChecker.hpp" // simple validity checker
// #include "ValidityCheckers/EuclideanScenario2ValidityChecker.hpp" // simple validity checker

#include "ValidityCheckers/state_validity_checker_pcc_blackmore.hpp" // simple validity checker
#include "ValidityCheckers/state_validity_checker_pcc_blackmore_euclidean.hpp" // simple validity checker
// #include "Planners/mod_sst.hpp" //PLANNER
#include "Planners/mod_rrt.hpp" //PLANNER
#include "StatePropagators/SimpleStatePropagator.h" //SIMPLE STATE PROPAGATOR *mostly done*
#include "StatePropagators/SimpleStatePropagatorEuclidean.h" //SIMPLE STATE PROPAGATOR *mostly done*
#include "Spaces/R2BeliefSpace.h" //R2 Belief Space *done*
#include "Spaces/R2BeliefSpaceEuclidean.h"
#include "Spaces/RNBeliefSpace.h"
#include "OptimizationObjectives/state_cost_objective.hpp"

#include "euclidean_main.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


ob::StateSpacePtr constructStateSpace(void)
{
    Eigen::MatrixXd sigma_init = 5.0 * Eigen::MatrixXd::Identity(2, 2);
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new RNBeliefSpace(2, sigma_init));
    return state_space;
}

ob::StateSpacePtr constructRealVectorStateSpace(void)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
    return state_space;
}

// Replace the MyGoalRegion class (lines 61-78) with the BeliefGoalRegion from barrier_rrt_main:
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

        // std::cout << state_belief->getX() << " " << state_belief->getY() << " " << goal_belief->getX() << " " << goal_belief->getY() << std::endl;
        
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


EuclideanMain::EuclideanMain(const std::string& config_file)
{
    //=======================================================================
    // Load parameters from config file
    //=======================================================================
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
        solving_time_ = 60.0;  // Default 60 seconds
        Q_noise_ = 0.2;
        R_noise_ = 0.1;
        R_bad_ = 5.0;
        K_default_ = 0.9;
    }
}

void EuclideanMain::loadConfig(const std::string& config_file)
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
    
    // Load scene name
    scene_name_ = pt.get<std::string>("Scene.scene");

    // Load planner parameters
    solving_time_ = pt.get<double>("Planner.planning_time");
}

void EuclideanMain::SaveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string stringpath)
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

        double x_pose = s->as<RNBeliefSpace::StateType>()->getX();
        double y_pose = s->as<RNBeliefSpace::StateType>()->getY();
        Mat cov = s->as<RNBeliefSpace::StateType>()->getCovariance();

        outputsolution << x_pose << ","  << y_pose << "," <<  cov.trace() << std::endl;

    }
    outputsolution << "10.0,10.0,10.0" << std::endl;

    outputsolution.close();

}


void EuclideanMain::planWithSimpleSetup()
{

    std::cout << "solving with scene: " << scene_name_ << std::endl;
    //=======================================================================
    // Instantiate the state space (SE2)
    //=======================================================================
    ob::StateSpacePtr space(constructStateSpace()); //space should probably be a class attribute
    // set the bounds for the R^2 part of RNBeliefSpace();
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(planning_bounds_x_[0]);
    bounds_se2.setHigh(planning_bounds_x_[1]);
    bounds_se2.setLow(planning_bounds_y_[0]);
    bounds_se2.setHigh(planning_bounds_y_[1]);
    
    space->as<RNBeliefSpace>()->setBounds(bounds_se2);
    
    //=======================================================================
    // Instantiate the control space
    //=======================================================================
    // oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(realspace, 2));
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
    // auto cspace(std::make_shared<R2BeliefSpace::StateType>(space, 2));

    ob::RealVectorBounds bounds(3);
    // bounds_se2.setLow(0, 0.0);
    bounds.setLow(0, -100.0);
    bounds.setHigh(0, 100.0);
    bounds.setLow(1, -100.0);
    bounds.setHigh(1, 100.0);
    bounds.setLow(2, 0.0);
    bounds.setHigh(2, 0.6);
    
    cspace->setBounds(bounds);
    
    //=======================================================================
    // Define a simple setup class
    //=======================================================================
    
    // simple_setup_ = oc::SimpleSetupPtr( new oc::SimpleSetup(cspace) );
    // oc::SpaceInformationPtr si = simple_setup_->getSpaceInformation();

    oc::SpaceInformationPtr si(new oc::SpaceInformation(space, cspace));
    
    // Set minimum and maximum duration of control action
    // si->setMinMaxControlDuration(min_control_duration_, max_control_duration_);
    si->setMinMaxControlDuration(1, 5);
    si->setPropagationStepSize(0.1);

    //=======================================================================
    // Create a planner for the defined space
    //=======================================================================
    // SST
    double goal_bias_ = 0.05;
    double sampling_bias_ = 0.20;
    double selection_radius_ = 5.0;
    double pruning_radius_ = 1.0;

    ob::PlannerPtr planner;
    planner = ob::PlannerPtr(new oc::mod_RRT(si));
    planner->as<oc::mod_RRT>()->setGoalBias(goal_bias_);
    planner->as<oc::mod_RRT>()->setSamplingBias(sampling_bias_);
    planner->as<oc::mod_RRT>()->setDistanceFunction(1); //1 is for wasserstein
    
    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    // create a start state
    ob::ScopedState<> start(space);
    start[0] = double(start_configuration_[0]); //x
    start[1] = double(start_configuration_[1]); //y
    // create a goal state

    ob::ScopedState<> goal(space);
    goal[0] = double(goal_configuration_[0]); //x
    goal[1] = double(goal_configuration_[1]); //y

    //=======================================================================
    // set the propagation routine for this space
    //=======================================================================

    // std::cout << "a" << std::endl;
    std::vector<std::vector<double>> measurement_regions = {
        {planning_bounds_x_[0], planning_bounds_x_[1]}, 
        {planning_bounds_y_[0], planning_bounds_y_[1]}
    };
    si->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si, Q_noise_, R_noise_, R_bad_, K_default_, measurement_regions)));
//	//=======================================================================
//	// Set optimization objective
//	//=======================================================================
//	//path length Objective
	
   

    // simple_setup_->print(std::cout);
    // simple_setup_->setup();
//  
//	//path clearance Objective
//	//simple_setup_->getProblemDefinition()->setOptimizationObjective(getClearanceObjective(si));
//
//	//path length and clearance Objective
//	//simple_setup_->getProblemDefinition()->setOptimizationObjective(getBalancedObjective1(si));
//
//	//path length and clearance Objective
//	//simple_setup_->getProblemDefinition()->setOptimizationObjective(getBalancedObjective2(si));
    ob::StateValidityCheckerPtr om_stat_val_check;
    // om_stat_val_check = ob::StateValidityCheckerPtr(new Scenario2ValidityChecker(si));
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore(scene_name_, si, 0.99, 0));
    // simple_setup_->setStateValidityChecker(om_stat_val_check);
    si->setStateValidityChecker(om_stat_val_check);

    si->setup();

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<BeliefGoalRegion>(si, goal.get(), 10.0));

    pdef->setOptimizationObjective(getEuclideanPathLengthObjective(si));
    pdef->getOptimizationObjective()->setCostThreshold(ob::Cost(45.0));
    planner->setProblemDefinition(pdef);


    ob::PlannerData planner_data(si);
    std::vector<double > costs;
    planner->setup();
    // planner->solve(0.01);
    // planner->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    std::ofstream outputfile;
    
    
    // for(unsigned int i = 1; i < planner_data.numVertices(); ++i) {

    //     double x = planner_data.getVertex(i).getState()->as<R2BeliefSpaceEuclidean::StateType>()->getX();
    //     double y = planner_data.getVertex(i).getState()->as<R2BeliefSpaceEuclidean::StateType>()->getY();
    //     Mat cov = planner_data.getVertex(i).getState()->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance();

    //     // std::cout << i << " " << x << " " << y << " " << cov.trace() << std::endl; 
    //     if (planner_data.getIncomingEdges(i, edgeList) > 0){
    //         outputfile << i << "," << edgeList[0] << "," << x << "," << y << "," << cov.trace() << "," << planner_data.getVertex(i).getState()->as<RNBeliefSpace::StateType>()->getCost() << std::endl; 
    //     }

        
    //     // for (int j = 0; j < edgeList.size(); ++j){
    //         // std::cout << edgeList[0] << std::endl;
    //         // std::cout << planner_data.getVertex(edgeList[0]).getState()->as<R2BeliefSpace::StateType>()->getX() << std::endl;
    //         // std::cout << planner_data.getVertex(i).getParent()->getState()->as<R2BeliefSpace::StateType>()->getX() std::endl;
    //     // }
    // }

    // outputfile.close();

    // if (planner->getProblemDefinition()->hasExactSolution()){
    //     const ompl::base::PathPtr &path = planner->getProblemDefinition()->getSolutionPath();              // convert to a generic path ptr
    //     oc::PathControl path_control = static_cast<oc::PathControl&>(*path);
    //     this->SaveSolutionPath(path_control, space, "solution_first.csv");
    // }
    // else{
    //     std::cout << "No solution for 0.001 secs" << std::endl;
    // }
    // planner->solve(1.0);
    // planner->getPlannerData(planner_data);

    // outputfile.open("tree_1sec.csv", std::ios::out | std::ios::trunc);
    // outputfile << "to,from,x,y,cov,cost" << std::endl;
    
    // for(unsigned int i = 1; i < planner_data.numVertices(); ++i) {

    //     double x = planner_data.getVertex(i).getState()->as<R2BeliefSpaceEuclidean::StateType>()->getX();
    //     double y = planner_data.getVertex(i).getState()->as<R2BeliefSpaceEuclidean::StateType>()->getY();
    //     Mat cov = planner_data.getVertex(i).getState()->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance();

    //     // std::cout << i << " " << x << " " << y << " " << cov.trace() << std::endl; 
    //     if (planner_data.getIncomingEdges(i, edgeList) > 0){
    //         outputfile << i << "," << edgeList[0] << "," << x << "," << y << "," << cov.trace() << "," << planner_data.getVertex(i).getState()->as<RNBeliefSpace::StateType>()->getCost() << std::endl; 
    //     }

    //     // for (int j = 0; j < edgeList.size(); ++j){
    //         // std::cout << edgeList[0] << std::endl;
    //         // std::cout << planner_data.getVertex(edgeList[0]).getState()->as<R2BeliefSpace::StateType>()->getX() << std::endl;
    //         // std::cout << planner_data.getVertex(i).getParent()->getState()->as<R2BeliefSpace::StateType>()->getX() std::endl;
    //     // }
    // }

    // outputfile.close();

    // if (planner->getProblemDefinition()->hasExactSolution()){
    // const ompl::base::PathPtr &path_1sec = planner->getProblemDefinition()->getSolutionPath(); 
    // oc::PathControl path_control = static_cast<oc::PathControl&>(*path_1sec);
    // this->SaveSolutionPath(path_control, space, "solution_1sec.csv");
    // }
    // else{
    //     std::cout << "No solution for 0.001 secs" << std::endl;
    // }
    planner->solve(solving_time_);
    planner->getPlannerData(planner_data);

    
    outputfile.open("solution_states_gbt.csv", std::ios::out | std::ios::trunc);
    outputfile << "x,y,cov,cost" << std::endl;
    
    for(unsigned int i = 1; i < planner_data.numVertices(); ++i) {

        double x = planner_data.getVertex(i).getState()->as<RNBeliefSpace::StateType>()->getX();
        double y = planner_data.getVertex(i).getState()->as<RNBeliefSpace::StateType>()->getY();
        Mat cov = planner_data.getVertex(i).getState()->as<RNBeliefSpace::StateType>()->getCovariance();

        // std::cout << i << " " << x << " " << y << " " << cov.trace() << std::endl; 
        if (planner_data.getIncomingEdges(i, edgeList) > 0){
            outputfile << i << "," << edgeList[0] << "," << x << "," << y << "," << cov.trace() << "," << planner_data.getVertex(i).getState()->as<RNBeliefSpace::StateType>()->getCost() << std::endl; 
        }
    }

    outputfile.close();

    if (planner->getProblemDefinition()->hasExactSolution()){
    const ompl::base::PathPtr &path_5sec = planner->getProblemDefinition()->getSolutionPath(); 
    oc::PathControl path_control = static_cast<oc::PathControl&>(*path_5sec);
    this->SaveSolutionPath(path_control, space, "solution_states_gbt.csv");
    }
    else{
        std::cout << "No solution for 10 secs" << std::endl;
    }
}

void EuclideanMain::solve(ob::PlannerPtr planner)
{
    //=======================================================================
    // Set state validity checking for this space
    //=======================================================================
    

    //=======================================================================
    // Attempt to solve the problem
    //=======================================================================
    if(simple_setup_->haveExactSolutionPath())
    {

        std::cout << "path found!" << std::endl;
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        //		simple_setup_->simplifySolution();
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

            double x_pose = s->as<RNBeliefSpace::StateType>()->getX();
            double y_pose = s->as<RNBeliefSpace::StateType>()->getY();
            Mat x_cov = s->as<RNBeliefSpace::StateType>()->getCovariance();
            std::cout << "x_pose: " << x_pose << std::endl;
            std::cout << "y_pose: " << y_pose << std::endl;
            std::cout << "cov: " << x_cov << std::endl;
            // //=======================================================================
            // // Plot covariance
            // //=======================================================================
            // std::cout << i << " " << x_pose << " " << y_pose Don't forget to leave your raid group first or you won't be able to click on the bone! Took me a minute or two to figure that one out :D.
            // visualizeCovariance(x_pose, y_pose, yaw, x_cov, y_cov, i);

            // std::cout << "cov " <<std::endl;
            // int cov_ind = 0;
            // for(int i=0; i<4 ; i++)
            // {
            //     for(int j=0; j<4 ; j++)
            //     {
            //         std::cout << s->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2)->values[cov_ind] << ", ";
            //         cov_ind++;
            //     }
            //     std::cout << std::endl;
            // }
        }

        oc::SpaceInformationPtr si = simple_setup_->getSpaceInformation();
        // std::cout << "distance is: " << space->as<RNBeliefSpace>()->distance(path_control_states[0], path_control_states[1]) << std::endl;
        // ob::StateSpacePtr space(constructStateSpace()); 
        // std::cout << "distance is: " << space->as<RNBeliefSpace>()->distance(path_control_states[0], path_control_states[1]) << std::endl;
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
    std::string config_file;
    
    if (argc > 1) {
        config_file = argv[1];
        std::cout << "Loading configuration from: " << config_file << std::endl;
    } else {
        std::cout << "No config file provided, using default values" << std::endl;
    }
    
    EuclideanMain offline_planner_uncertainty(config_file);
    std::cout << "Starting planning..." << std::endl;
    offline_planner_uncertainty.planWithSimpleSetup();

    return 0;
}
