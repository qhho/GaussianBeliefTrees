/*
 */

#include <iostream>
#include <vector>
#include <boost/bind.hpp>

// headers
#include "ValidityCheckers/state_validity_checker_pcc_blackmore.hpp" // simple validity checker
#include "StatePropagators/SimpleStatePropagator.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R2BeliefSpaceEuclidean.h"
// #include "ControlSpaces/SimpleControlSpace.h"
#include "OptimizationObjectives/state_cost_objective.hpp"
#include "OptimizationObjectives/expected_cost_objective.hpp"
#include <boost/program_options/parsers.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <string>
#include <bits/stdc++.h>
#include "benchmark_tro_2dsimple.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

std::vector<std::string> split(std::string str, std::string delimiter)
{
   std::vector<std::string> v;
    if (!str.empty()) {
        int start = 0;
        do {
            // Find the index of occurrence
            int idx = str.find(delimiter, start);
            if (idx == std::string::npos) {
                break;
            }
 
            // If found add the substring till that
            // occurrence in the vector
            int length = idx - start;
            v.push_back(str.substr(start, length));
            start += (length + delimiter.size());
        } while (true);
        v.push_back(str.substr(start));
    }
 
    return v;
}

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
    MyGoalRegion(const SpaceInformationPtr &si, double p_safe, std::vector<double> goal_state, double goal_r, double t_crit) : ompl::base::GoalRegion(si)
    {
        setThreshold(goal_r);
        goal_state_ = goal_state;
        t_crit_ = t_crit;
        p_safe_ = p_safe;
    }
    virtual double distanceGoal(const Belief *st) const
        {
        double dx = st->x;
        double dy = st->y;
        double radius = st->sigma_(0,0) + st->lambda_(0,0);
        radius = t_crit_*sqrt(radius);

        // std::cout << sqrt(dx*dx + dy*dy) + radius << std::endl;
        return sqrt(dx*dx + dy*dy) + radius;
        // return sqrt(dx*dx + dy*dy);
    }

    //TODO: add distance to goal for whole motion node.

    double p_safe_;
    std::vector<double> goal_state_;
    double goal_r_;
    double t_crit_;
};

void OfflinePlannerUncertainty::planWithSimpleSetup(int sysType, double plan_time, double dt, double p_safe, double Q, double R, double K, std::string scene,  std::vector<std::vector<double>> measurement_region, std::vector<std::vector<double>> bounds_state, std::vector<std::vector<double>> bounds_control, std::vector< double> goal_state, double goal_r, std::vector< double> initial_state, double goal_bias, double selection_radius, double pruning_radius, double sampling_bias, double control_duration_low, double control_duration_high, std::string file)
{
    //=======================================================================
    // Instantiate the state space
    //=======================================================================
    ob::StateSpacePtr space(constructRealVectorStateSpace()); // [x y surge yaw covariance(16)] -> size = 20
    // set the bounds for the R^2 part of R2BeliefSpace();
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, bounds_state[0][0]);
    bounds_se2.setHigh(0, bounds_state[0][1]);
    bounds_se2.setLow(1, bounds_state[1][0]);
    bounds_se2.setHigh(1, bounds_state[1][1]);
    space->as<R2BeliefSpace>()->setBounds(bounds_se2);
    

    std::cout << bounds_state[0][0] << " " << bounds_state[0][1] << " " << bounds_state[1][0] << " " << bounds_state[1][1] << std::endl;

    //=======================================================================
    // Instantiate the control space
    //=======================================================================
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, bounds_control[0][0]);
    bounds.setHigh(0, bounds_control[0][1]);
    bounds.setLow(1, bounds_control[0][0]);
    bounds.setHigh(1, bounds_control[0][1]);
    cspace->setBounds(bounds);
    
    //=======================================================================
    // Define a simple setup class
    //=======================================================================
    
    simple_setup_ = oc::SimpleSetupPtr( new oc::SimpleSetup(cspace) );
    oc::SpaceInformationPtr si = simple_setup_->getSpaceInformation();
    
    // Set minimum and maximum duration of control action
    si->setMinMaxControlDuration(control_duration_low, control_duration_high);
    si->setPropagationStepSize(dt);

    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    ob::ScopedState<> start(space);
    start[0] = double(initial_state[0]); //x
    start[1] = double(initial_state[1]); //y

    simple_setup_->setStartState(start);
    double t_crit = quantile(boost::math::chi_squared(2), p_safe);
    simple_setup_->setGoal(std::make_shared<MyGoalRegion>(si, p_safe, goal_state, goal_r, t_crit));
    //=======================================================================
    // set the propagation routine for this space
    //=======================================================================
    simple_setup_->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagator(si, Q, R, K, measurement_region)));
//	//=======================================================================
//	// Set optimization objective
//	//=======================================================================
//	//path length Objective
	simple_setup_->getProblemDefinition()->setOptimizationObjective(getExpectedPathLengthObjective(si));
    ob::StateValidityCheckerPtr val_checker;
    val_checker = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore(scene, simple_setup_->getSpaceInformation(), p_safe));
    simple_setup_->setStateValidityChecker(val_checker);

    //=======================================================================
    // Perform setup steps for the planner
    //=======================================================================
    simple_setup_->setup();
    OMPL_INFORM("Benchmarking starting");
    this->solve(goal_bias, sampling_bias, selection_radius, pruning_radius, 1, file);
}

void OfflinePlannerUncertainty::solve(double goal_bias, double sampling_bias, double selection_radius, double pruning_radius, int distance_function, std::string file)
{
    //=======================================================================
    // Set state validity checking for this space
    //=======================================================================

    simple_setup_->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(00));
    ompl::tools::MyBenchmarkRRBT b(*simple_setup_, "1 rrbt");
    
    // // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
    b.addExperimentParameter("measurementregion", "x y", "55 25");
    b.addExperimentParameter("first_solution", "bool", "1");
    std::string resultsfile = "../results/rrbt/" + file + "/";
    b.setDir(resultsfile);
    // // b.addExperimentParameter("num_obstacles", "INTEGER", "10");
    
    // // We add the planners to evaluate.
    // b.addPlanner(ob::PlannerPtr(new oc::mod_RRT(simple_setup_->getSpaceInformation())));
    double goal_bias_ = goal_bias;

    ob::PlannerPtr RRBT_Planner;
    RRBT_Planner = ob::PlannerPtr(new oc::RRBT(simple_setup_->getSpaceInformation()));
    RRBT_Planner->as<oc::RRBT>()->setGoalBias(goal_bias_);
    RRBT_Planner->as<oc::RRBT>()->setRange(10.0); //TODO: fix this.
    b.addPlanner(RRBT_Planner);
    
    // Now we can benchmark: 5 second time limit for each plan computation,
    // 100 MB maximum memory usage per plan computation, 50 runs for each planner
    // and true means that a text-mode progress bar should be displayed while
    // computation is running.
    ompl::tools::MyBenchmarkRRBT::MyRequest req;
    req.maxTime = 50;
    req.maxMem = 10000.0;
    req.runCount = 100;
    req.displayProgress = true;
    b.benchmark(req);
    // // This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
}

int main(int argc, char **argv)
{

    std::string configtype = argv[1];
    std::string inifile = "../config/" + configtype + ".ini";
    std::string file = configtype;
    // Read parameters from config.ini file
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(inifile, pt);

    // System params
    int sysType = std::stoi(pt.get<std::string>("System.sysType"));
    // int discTime = std::stoi(pt.get<std::string>("System.discTime"));
    double Q = std::stod(pt.get<std::string>("System.Q"));
    double R = std::stod(pt.get<std::string>("System.R"));
    double K = std::stod(pt.get<std::string>("System.K"));
    double dt = std::stod(pt.get<std::string>("System.dt"));
    std::string scene = pt.get<std::string>("Scene.scene");

    // Planner params
    double p_safe = std::stod(pt.get<std::string>("Planner.p_safe"));
    double plan_time = std::stod(pt.get<std::string>("Planner.planning_time"));
    std::string goal = pt.get<std::string>("Planner.goal");
    std::vector<std::string> spltStr = split(goal, ",");
    double goal_x = std::stod(spltStr[0]);
    double goal_y = std::stod(spltStr[1]);
    double goal_r = std::stod(pt.get<std::string>("Planner.goal_radius"));
    std::string init = pt.get<std::string>("Planner.initial_state");
    spltStr = split(init, ",");
    double initial_state_x = std::stod(spltStr[0]);
    double initial_state_y = std::stod(spltStr[1]);

    double pruning_radius = std::stod(pt.get<std::string>("Planner.pruning_radius"));
    double selection_radius = std::stod(pt.get<std::string>("Planner.selection_radius"));
    double sampling_bias = std::stod(pt.get<std::string>("Planner.sampling_bias"));

    double goal_bias = std::stod(pt.get<std::string>("Planner.goal_bias"));

    std::string control_duration_str = pt.get<std::string>("Planner.control_duration");
    spltStr = split(control_duration_str, ",");
    double control_duration_low = std::stod(spltStr[0]);
    double control_duration_high = std::stod(spltStr[1]);

    // Environment params
    // x,y bounds for unicycle system
    std::string bounds_x = pt.get<std::string>("Environment.x_bounds");
    spltStr = split(bounds_x, ",");
    double bounds_x_low = std::stod(spltStr[0]);
    double bounds_x_high = std::stod(spltStr[1]);

    std::string bounds_y = pt.get<std::string>("Environment.y_bounds");
    spltStr = split(bounds_y, ",");
    double bounds_y_low = std::stod(spltStr[0]);
    double bounds_y_high = std::stod(spltStr[1]);
    // Bounds on states
    std::string Boundstr = pt.get<std::string>("Environment.state_bounds");
    std::vector<std::string> Boundsplt = split(Boundstr, ":"); // Split rows
    std::vector<std::string>::iterator iterBound = Boundsplt.begin(); // Iterate through and build vector of doubles
    std::vector< std::vector<double>> bounds_state;
    for(iterBound; iterBound < Boundsplt.end(); iterBound++)
    {
        std::vector<std::string> spltStrBound_rows = split(*iterBound, ","); // Split rows
        std::vector<double> rowsBound_doub(spltStrBound_rows.size());
        std::transform(spltStrBound_rows.begin(), spltStrBound_rows.end(), rowsBound_doub.begin(), [](const std::string& val)
            {
                return std::stod(val);
            });
        bounds_state.push_back(rowsBound_doub);
    }

    // Bounds on controls
    Boundstr = pt.get<std::string>("Environment.control_bounds");
    Boundsplt = split(Boundstr, ":"); // Split rows
    iterBound = Boundsplt.begin(); // Iterate through and build vector of doubles
    std::vector< std::vector<double>> bounds_control;
    for(iterBound; iterBound < Boundsplt.end(); iterBound++)
    {
        std::vector<std::string> spltStrBound_rows = split(*iterBound, ","); // Split rows
        std::vector<double> rowsBound_doub(spltStrBound_rows.size());
        std::transform(spltStrBound_rows.begin(), spltStrBound_rows.end(), rowsBound_doub.begin(), [](const std::string& val)
            {
                return std::stod(val);
            });
        bounds_control.push_back(rowsBound_doub);
    }

    Boundstr = pt.get<std::string>("Environment.measurement_region");
    Boundsplt = split(Boundstr, ":"); // Split rows
    iterBound = Boundsplt.begin(); // Iterate through and build vector of doubles
    std::vector< std::vector<double>> measurement_region;
    for(iterBound; iterBound < Boundsplt.end(); iterBound++)
    {
        std::vector<std::string> spltStrBound_rows = split(*iterBound, ","); // Split rows
        std::vector<double> rowsBound_doub(spltStrBound_rows.size());
        std::transform(spltStrBound_rows.begin(), spltStrBound_rows.end(), rowsBound_doub.begin(), [](const std::string& val)
            {
                return std::stod(val);
            });
        measurement_region.push_back(rowsBound_doub);
    }

    OMPL_INFORM("USING P_SAFE: %3f, Q: %3f, R: %3f", p_safe, Q, R);
    if(sysType==1){
        OMPL_INFORM("Using linearized unicycle system");
    } else if (sysType==0){
        OMPL_INFORM("Using linear system");
    }
    else{
        OMPL_ERROR("Invalid system type");
    }

    std::vector< double > goal_state = {goal_x, goal_y};
    std::vector< double > initial_state = {initial_state_x, initial_state_y};

    OfflinePlannerUncertainty offline_planner_uncertainty;
    offline_planner_uncertainty.planWithSimpleSetup(sysType, plan_time, dt, p_safe, Q, R, K, scene, measurement_region, bounds_state, bounds_control, goal_state, goal_r, initial_state, goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, control_duration_high, file);

    return 0;
}