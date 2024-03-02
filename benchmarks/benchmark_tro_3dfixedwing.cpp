/*
 */

#include <iostream>
#include <vector>
#include <boost/bind.hpp>

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
#include "ValidityCheckers/Scenario1ValidityChecker.hpp" // simple validity checker
#include "ValidityCheckers/Scenario2ValidityChecker.hpp" // simple validity checker
#include "ValidityCheckers/Scenario3ValidityChecker.hpp" // simple validity checker
#include "ValidityCheckers/state_validity_checker_pcc_blackmore.hpp" // simple validity checker

// #include "Planners/mod_sst.hpp" //PLANNER
#include "StatePropagators/SimpleStatePropagator.h" //SIMPLE STATE PROPAGATOR *mostly done*
#include "Spaces/R2BeliefSpace.h" //R2 Belief Space *done*
#include "Spaces/R2BeliefSpaceEuclidean.h"
#include "ControlSpaces/SimpleControlSpace.h"  // STEER FUNCTION
#include "OptimizationObjectives/state_cost_objective.hpp"

#include "benchmark_main.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

ob::StateSpacePtr constructStateSpace(void)
{
    ob::StateSpacePtr state_space = ob::StateSpacePtr(new R2BeliefSpace());
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
    MyGoalRegion(const SpaceInformationPtr &si) : ompl::base::GoalRegion(si)
    {
        setThreshold(16.0);
    }
 
    virtual double distanceGoal(const State *st) const
        {

        double dx = st->as<R2BeliefSpace::StateType>()->getX() - 90.0;
        double dy = st->as<R2BeliefSpace::StateType>()->getY() - 90.0;

        return (dx*dx + dy*dy);
        // perform any operations and return a double indicating the distance to the goal
    }
};


//!  OfflinePlannerUncertainty class.
/*!
 * Offline Planner.
 * Setup an SST for offline computation of collision-free paths.
 * C-Space: R2BeliefSpace
*/
// class OfflinePlannerUncertainty
// {
//     public:
//         //! Constructor
//         OfflinePlannerUncertainty();
//         void planWithSimpleSetup();
//     private:

//         // OMPL
//         oc::SimpleSetupPtr simple_setup_;
//         double planning_depth_, watchdog_period_, solving_time_, min_control_duration_, max_control_duration_, accep_prob_;
//         bool lazy_collision_eval_;
//         std::vector<double> planning_bounds_x_, planning_bounds_y_, start_configuration_, goal_configuration_, initial_covariance_;
//         std::string planner_name_;
//         std::vector<const ob::State *> path_states_;
//         std::vector<const oc::Control *> path_controls_;
// };

// double mydistancefunction(const State *state1, const State *state2) const
//             {
//                 return stateSpace_->distance(state1, state2);
//             }

OfflinePlannerUncertainty::OfflinePlannerUncertainty()
{
    //=======================================================================
    // Get parameters
    //=======================================================================
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_configuration_.resize(2);
    goal_configuration_.resize(2);

    planning_bounds_x_[0] = 0.0;
    planning_bounds_x_[1] = 100.0;
    planning_bounds_y_[0] = 0.0;
    planning_bounds_y_[1] = 100.0;

    start_configuration_[0] = 10.0; // 50.0; //15.0;
    start_configuration_[1] = 10.0; // 10.0; //50.0;
    goal_configuration_[0] = 90.0;
    goal_configuration_[1] = 90.0;

    initial_covariance_ = 1.0*Eigen::MatrixXd::Identity(2, 2);
}

void OfflinePlannerUncertainty::planWithSimpleSetup()
{
    //=======================================================================
    // Instantiate the state space (SE2)
    //=======================================================================
    ob::StateSpacePtr space(constructStateSpace()); // [x y surge yaw covariance(16)] -> size = 20
    // set the bounds for the R^2 part of R2BeliefSpace();
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, 0.0);
    bounds_se2.setHigh(0, 100.0);
    bounds_se2.setLow(1, 0.0);
    bounds_se2.setHigh(1, 100.0);
    
    space->as<R2BeliefSpace>()->setBounds(bounds_se2);
    
    //=======================================================================
    // Instantiate the control space
    //=======================================================================
    // oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(realspace, 2));
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    // auto cspace(std::make_shared<R2BeliefSpace::StateType>(space, 2));

    ob::RealVectorBounds bounds(2);
    // bounds_se2.setLow(0, 0.0);
    bounds.setLow(-100.0);
    // bounds_se2.setigh(0, 100.0);
    // bounds_se2.setLow(1, 0.0);
    bounds.setHigh(100.0);
    
    cspace->setBounds(bounds);
    
    //=======================================================================
    // Define a simple setup class
    //=======================================================================
    
    simple_setup_ = oc::SimpleSetupPtr( new oc::SimpleSetup(cspace) );
    oc::SpaceInformationPtr si = simple_setup_->getSpaceInformation();
    
    // Set minimum and maximum duration of control action
    // si->setMinMaxControlDuration(min_control_duration_, max_control_duration_);
    si->setMinMaxControlDuration(1, 10);
    si->setPropagationStepSize(0.1);

    //=======================================================================
    // Create a planner for the defined space
    //=======================================================================
    ob::PlannerPtr planner;

    double goal_bias_ = 0.05;
    double selection_radius_ = 5.0;
    double pruning_radius_ = 1.5; //0.5 sucks, but 1.5 good

    // planner = ob::PlannerPtr(new oc::SST(si));
    // planner->as<oc::SST>()->setGoalBias(goal_bias_);
    // planner->as<oc::SST>()->setSelectionRadius(selection_radius_);
    // planner->as<oc::SST>()->setPruningRadius(pruning_radius_);
    
    // planner = ob::PlannerPtr(new oc::RRT(si));
    // auto planner(std::make_shared<oc::RRT>(si));
    // ob::PlannerPtr planner(new ompl::control::RRT(si));
    // planner->as<oc::RRT>()->setGoalBias(goal_bias_);
    // planner->as<oc::RRT>()->setRange(15.0);
    // planner = ob::PlannerPtr(new oc::RRT(si));


    // planner = ob::PlannerPtr(new oc::EST(si));
    // planner->as<oc::EST>()->setRange(15.0);

    //=======================================================================
    // Setup the setup planner
    //=======================================================================
    // simple_setup_->setPlanner(planner);

    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    // create a start state
    ob::ScopedState<> start(space);
    start[0] = double(start_configuration_[0]); //x
    start[1] = double(start_configuration_[1]); //y
    // create a goal state

    // ob::ScopedState<> goal(space);
    // goal[0] = double(goal_configuration_[0]); 	//x

    // goal[1] = double(goal_configuration_[1]); 	//y

    //=======================================================================
    // Set the start and goal states
    //=======================================================================
    // simple_setup_->setStartAndGoalStates(start, goal, 20.0);
    simple_setup_->setStartState(start);
    simple_setup_->setGoal(std::make_shared<MyGoalRegion>(si));
    //=======================================================================
    // set the propagation routine for this space
    //=======================================================================
    simple_setup_->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si)));
//	//=======================================================================
//	// Set optimization objective
//	//=======================================================================
//	//path length Objective
	simple_setup_->getProblemDefinition()->setOptimizationObjective(getEuclideanPathLengthObjective(si));
    // simple_setup_->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(1000.0));
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
    om_stat_val_check = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore("scene4", simple_setup_->getSpaceInformation(), 0.95));
    simple_setup_->setStateValidityChecker(om_stat_val_check);

    //=======================================================================
    // Perform setup steps for the planner
    //=======================================================================
    simple_setup_->setup();
    //simple_setup_->print();
    std::cout << "benchmarking" << std::endl;
    this->solve();
}

void OfflinePlannerUncertainty::solve()
{
    //=======================================================================
    // Set state validity checking for this space
    //=======================================================================
    
    //=======================================================================
    // Attempt to solve the problem
    //=======================================================================

    //experiment name is SCENARIONUMBER_DISTANCEFUNCTION_BIASVALUE
    double times[3] =  {0.0, 5.0, 10.0};
    double biastrials[2] = {0.0, 0.20};
    for (int k = 0; k < 20; ++k){
        for (int i = 2; i < 3; i++){

        //experiment name is SCENARIONUMBER_DISTANCEFUNCTION_BIASVALUE
        simple_setup_->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(0.0));
        ompl::tools::MyBenchmark b(*simple_setup_, "4 wasserstein");
        
        // // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
        // b.addExperimentParameter("bias_value", "double", goal_bias_);
        // b.addExperimentParameter("measurementregion", "x y", "55 25");
        b.addExperimentParameter("first_solution", "bool", "0");
        b.setDir("results/wasserstein/scenario1final/");
        // // b.addExperimentParameter("num_obstacles", "INTEGER", "10");
        
        // // We add the planners to evaluate.
        double goal_bias_ = 0.05;
        double sampling_bias_ = 0.05 * k;

        b.addExperimentParameter("bias_value", "double", std::to_string(sampling_bias_));
        // ob::PlannerPtr RRG_Planner;
        // RRG_Planner = ob::PlannerPtr(new oc::RRG(simple_setup_->getSpaceInformation()));
        // RRG_Planner->as<oc::RRG>()->setGoalBias(goal_bias_);
        // RRG_Planner->as<oc::RRG>()->setSamplingBias(sampling_bias_);
        // RRG_Planner->as<oc::RRG>()->setDistanceFunction(1); //wasserstein
        // b.addPlanner(RRG_Planner);

        // ob::PlannerPtr EST_Planner;
        // EST_Planner = ob::PlannerPtr(new oc::EST(simple_setup_->getSpaceInformation()));
        // EST_Planner->as<oc::EST>()->setRange(10.0);
        // b.addPlanner(EST_Planner);

        // std::cout << "what" << std::endl;
        // ob::PlannerPtr RRBT_Planner;
        // RRBT_Planner = ob::PlannerPtr(new oc::RRBT(simple_setup_->getSpaceInformation()));
        // b.addPlanner(RRBT_Planner);

        // std::cout << "the" << std::endl;
        // // etc
        
        // // For planners that we want to configure in specific ways,
        // // the ompl::base::PlannerAllocator should be used:
        
        // SST
        
        double selection_radius_ = 5.0;
        double pruning_radius_ = 3.0; //0.5 sucks, but 1.5 good, 2.5 is better
        
        ob::PlannerPtr planner;
        planner = ob::PlannerPtr(new oc::SSBT(simple_setup_->getSpaceInformation()));
        planner->as<oc::SSBT>()->setGoalBias(goal_bias_);
        planner->as<oc::SSBT>()->setSelectionRadius(selection_radius_);
        planner->as<oc::SSBT>()->setPruningRadius(pruning_radius_);
        planner->as<oc::SSBT>()->setSamplingBias(sampling_bias_);
        planner->as<oc::SSBT>()->setDistanceFunction(1); //wasserstein
        b.addPlanner(planner);
        
        // Now we can benchmark: 5 second time limit for each plan computation,
        // 100 MB maximum memory usage per plan computation, 50 runs for each planner
        // and true means that a text-mode progress bar should be displayed while
        // computation is running.
        ompl::tools::MyBenchmark::MyRequest req;
        req.maxTime = times[i] + 1.0;
        req.maxMem = 1000.0;
        req.runCount = 50;
        req.displayProgress = true;
        b.benchmark(req);
        // // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile();
        }

        /*
        simple_setup_->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(1000.0));
        ompl::tools::MyBenchmark b(*simple_setup_, "4 wasserstein");
        
        // // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
        // b.addExperimentParameter("measurementregion", "x y", "55 25");
        b.addExperimentParameter("first_solution", "bool", "1");
        b.setDir("results/wasserstein/scenario4final/");
        // // b.addExperimentParameter("num_obstacles", "INTEGER", "10");
        
        // // We add the planners to evaluate.
        // b.addPlanner(ob::PlannerPtr(new oc::mod_RRT(simple_setup_->getSpaceInformation())));
        double goal_bias_ = 0.05;
        double sampling_bias_ = 0.05 * k;

        b.addExperimentParameter("bias_value", "double", std::to_string(sampling_bias_));

        ob::PlannerPtr RRG_Planner;
        RRG_Planner = ob::PlannerPtr(new oc::RRG(simple_setup_->getSpaceInformation()));
        RRG_Planner->as<oc::RRG>()->setGoalBias(goal_bias_);
        RRG_Planner->as<oc::RRG>()->setSamplingBias(sampling_bias_);
        RRG_Planner->as<oc::RRG>()->setDistanceFunction(1); //wasserstein
        b.addPlanner(RRG_Planner);

        ob::PlannerPtr EST_Planner;
        EST_Planner = ob::PlannerPtr(new oc::EST(simple_setup_->getSpaceInformation()));
        EST_Planner->as<oc::EST>()->setRange(10.0);
        b.addPlanner(EST_Planner);
        
        // SST
        
        double selection_radius_ = 5.0;
        double pruning_radius_ = 3.0; //0.5 sucks, but 1.5 good, 2.5 is better
        
        ob::PlannerPtr planner;
        planner = ob::PlannerPtr(new oc::SSBT(simple_setup_->getSpaceInformation()));
        planner->as<oc::SSBT>()->setGoalBias(goal_bias_);
        planner->as<oc::SSBT>()->setSelectionRadius(selection_radius_);
        planner->as<oc::SSBT>()->setPruningRadius(pruning_radius_);
        planner->as<oc::SSBT>()->setSamplingBias(sampling_bias_);
        planner->as<oc::SSBT>()->setDistanceFunction(1); //wasserstein
        b.addPlanner(planner);
        
        // Now we can benchmark: 5 second time limit for each plan computation,
        // 100 MB maximum memory usage per plan computation, 50 runs for each planner
        // and true means that a text-mode progress bar should be displayed while
        // computation is running.
        ompl::tools::MyBenchmark::MyRequest req;
        req.maxTime = 100;
        req.maxMem = 1000.0;
        req.runCount = 100;
        req.displayProgress = true;
        b.benchmark(req);
        // // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile();
        */
    }
}

int main(int argc, char **argv)
{

    OfflinePlannerUncertainty offline_planner_uncertainty;
    offline_planner_uncertainty.planWithSimpleSetup();

    return 0;
}