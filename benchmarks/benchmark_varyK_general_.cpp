/*
 * Modified to support custom A and B matrices for linear systems
 */

 #include <iostream>
 #include <vector>
 #include <boost/bind.hpp>
 
 // headers
 #include <boost/program_options/parsers.hpp>
 #include <boost/property_tree/ptree.hpp>
 #include <boost/property_tree/ini_parser.hpp>
 #include <boost/math/distributions/chi_squared.hpp>
 #include <string>
 #include <bits/stdc++.h>
 #include "benchmark_gbt.hpp"
 #include "StatePropagators/SimpleStatePropagatorfixedK.h"
 #include "StatePropagators/2DUnicyclePropagatorfixedK.h"
 #include "StatePropagators/3DSimpleStatePropagatorfixedK.h"
 #include "StatePropagators/3DUnicyclePropagatorfixedK.h"
 #include "StatePropagators/LinearStatePropagator.h" // Include the new propagator
 
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
 
 // Parse matrix strings in format "a,b,c,d:e,f,g,h:i,j,k,l:m,n,o,p"
 Eigen::MatrixXd parseMatrix(const std::string& matrixStr, int rows, int cols) {
     Eigen::MatrixXd matrix(rows, cols);
     
     // Split rows
     std::vector<std::string> rowStrings = split(matrixStr, ":");
     
     if (rowStrings.size() != rows) {
         OMPL_ERROR("Matrix row count mismatch: expected %d, got %d", rows, (int)rowStrings.size());
         return matrix;
     }
     
     for (int i = 0; i < rows; i++) {
         // Split columns
         std::vector<std::string> colStrings = split(rowStrings[i], ",");
         
         if (colStrings.size() != cols) {
             OMPL_ERROR("Matrix column count mismatch in row %d: expected %d, got %d", 
                       i, cols, (int)colStrings.size());
             return matrix;
         }
         
         for (int j = 0; j < cols; j++) {
             matrix(i, j) = std::stod(colStrings[j]);
         }
     }
     
     return matrix;
 }
 
 ob::StateSpacePtr constructStateSpace(int dim, bool euclidean)
 {
      ob::StateSpacePtr state_space;
     if (dim == 2)
         state_space = ob::StateSpacePtr(new R2BeliefSpace(5.0));
     else if (dim == 3)
         state_space = ob::StateSpacePtr(new R3BeliefSpace(2.0));
     else if (dim == 4) // For double integrator in 2D
         state_space = ob::StateSpacePtr(new RNBeliefSpace(dim, euclidean, 5.0 * Eigen::MatrixXd::Identity(dim, dim)));
     else
         OMPL_ERROR("Invalid dimension. Must be 2, 3, or 4");
     return state_space;
 }
 
 ob::StateSpacePtr constructUnicycleStateSpace(int dim)
 {
     ob::StateSpacePtr c_space = ob::StateSpacePtr(new ob::CompoundStateSpace());
     if (dim == 2)
     {
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new R2BeliefSpace(5.0)), 1.0); // x, y, P
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 0.0); // yaw
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0); // surge
         c_space->as<ob::CompoundStateSpace>()->lock();
     }
     else if (dim == 3)
     {
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new R3BeliefSpace(5.0)), 1.0); // x, y, z, P
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace()), 0.0); // yaw
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0); // surge
         c_space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0); // heave
         c_space->as<ob::CompoundStateSpace>()->lock();
     }
     else
         OMPL_ERROR("Invalid dimension. Must be 2 or 3");
     return c_space;
 }
 
 ob::StateSpacePtr constructRealVectorStateSpace(void)
 {
     ob::StateSpacePtr state_space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
     return state_space;
 }
 
 class MyGoalRegion : public ompl::base::GoalRegion
 {
 public:
     MyGoalRegion(const SpaceInformationPtr &si, double p_safe, std::vector<double> goal_state, double goal_r, double t_crit, double dim) : ompl::base::GoalRegion(si)
     {
         setThreshold(goal_r);
         goal_state_ = goal_state;
         t_crit_ = t_crit;
         p_safe_ = p_safe;
         dim_ = dim;
     }
     virtual double distanceGoal(const State *st) const
         {
     
         if (dim_ == 2)
         {
             
             double dx = st->as<R2BeliefSpace::StateType>()->getX() - goal_state_[0];
             double dy = st->as<R2BeliefSpace::StateType>()->getY() - goal_state_[1];
             double radius = st->as<R2BeliefSpace::StateType>()->getCovariance()(0,0);
             radius = t_crit_*sqrt(radius);
             return sqrt(dx*dx + dy*dy) + radius;
             
         }
         else if (dim_ == 4) // Double integrator
         {
             double dx = st->as<RNBeliefSpace::StateType>()->getComponent(0) - goal_state_[0]; // x position
             double dy = st->as<RNBeliefSpace::StateType>()->getComponent(1) - goal_state_[1]; // y position
            //  double dvx = st->as<RNBeliefSpace::StateType>()->getComponent(2) - goal_state_[2]; // x velocity
            //  double dvy = st->as<RNBeliefSpace::StateType>()->getComponent(3) - goal_state_[3]; // y velocity
             
             double position_dist = sqrt(dx*dx + dy*dy);
            //  double velocity_dist = sqrt(dvx*dvx + dvy*dvy);
             
             // Use position uncertainty (first diagonal element of covariance)
             double radius = st->as<RNBeliefSpace::StateType>()->getCovariance()(0,0);
             radius = t_crit_*sqrt(radius);
             
             // Weighted sum of position and velocity distances
             return position_dist + radius;
         }
         else
         {
             double dx = st->as<R2BeliefSpace::StateType>()->getX() - goal_state_[0];
             double dy = st->as<R2BeliefSpace::StateType>()->getY() - goal_state_[1];
             double dz = st->as<R3BeliefSpace::StateType>()->getZ() - goal_state_[2];
             double radius = st->as<R3BeliefSpace::StateType>()->getCovariance()(0,0);
             radius = t_crit_*sqrt(radius);
             return sqrt(dx*dx + dy*dy + dz*dz) + radius;
         }
         
         return 0;
     }
 
     double p_safe_;
     std::vector<double> goal_state_;
     double goal_r_;
     double t_crit_;
     int dim_;
 };
 
 class MyUnicycleGoalRegion : public ompl::base::GoalRegion
 {
 public:
     MyUnicycleGoalRegion(const SpaceInformationPtr &si, double p_safe, std::vector<double> goal_state, double goal_r, double t_crit,  double dim) : ompl::base::GoalRegion(si)
     {
         setThreshold(goal_r);
         goal_state_ = goal_state;
         t_crit_ = t_crit;
         p_safe_ = p_safe;
         dim_ = dim;
     }
     virtual double distanceGoal(const State *st) const
     {
         if (dim_ == 2)
         {
             double dx = st->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX() - goal_state_[0];
             double dy = st->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY() - goal_state_[1];
 
             double radius = st->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0);
             radius = t_crit_*sqrt(radius);
             return sqrt(dx*dx + dy*dy) + radius;
         }
         else
         {
             double dx = st->as<base::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getX() - goal_state_[0];
             double dy = st->as<base::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getY() - goal_state_[1];
             double dz = st->as<base::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getZ() - goal_state_[2];
             double radius = st->as<base::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getCovariance()(0,0);
             radius = t_crit_*sqrt(radius);
             return sqrt(dx*dx + dy*dy + dz*dz) + radius;
         }
         return 0;
     }
 
     double p_safe_;
     std::vector<double> goal_state_;
     double goal_r_;
     double t_crit_;
     int dim_;
 };
 
 
 void OfflinePlannerUncertainty::planWithUnicycle(int sysType, double plan_time, double dt, double p_safe, double Q, double R, double R_bad, double K, std::string scene,  std::vector<std::vector<double>> measurement_region, std::vector<std::vector<double>> bounds_state, std::vector<std::vector<double> > bounds_surge, std::vector<std::vector<double>> bounds_control, std::vector< double> goal_state, double goal_r, std::vector< double> initial_state, double goal_bias, double selection_radius, double pruning_radius, double sampling_bias, double control_duration_low, double control_duration_high, std::string file)
 {   
      int dimension;
 
     if (sysType == 2)
         dimension = 2;
     else if (sysType == 3)
         dimension = 3;
     else
         OMPL_ERROR("Invalid system type. Must be 2 or 3");
 
     ob::StateSpacePtr space(constructUnicycleStateSpace(dimension)); // [x y surge yaw covariance(16)] -> size = 20
     // set the bounds for the R^2 part of R2BeliefSpace();
     ob::RealVectorBounds bounds_se2(dimension);
 
     for (int i = 0; i < dimension; i++)
     {
         bounds_se2.setLow(i, bounds_state[i][0]);
         bounds_se2.setHigh(i, bounds_state[i][1]);
     }
 
     if (dimension == 2)
     {
         space->as<ob::CompoundStateSpace>()->as<R2BeliefSpace>(0)->setBounds(bounds_se2);
     }
     else if (dimension == 3)
     {
         space->as<ob::CompoundStateSpace>()->as<R3BeliefSpace>(0)->setBounds(bounds_se2);
         ob::RealVectorBounds bound_heave(1);
         bound_heave.setLow(bounds_surge[1][0]);
         bound_heave.setHigh(bounds_surge[1][1]);
         space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(3)->setBounds(bound_heave);
     }
 
     ob::RealVectorBounds bound_surge(1);
     bound_surge.setLow(bounds_surge[0][0]);
     bound_surge.setHigh(bounds_surge[0][1]);
     space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2)->setBounds(bound_surge);
     //=======================================================================
     // Instantiate the control space
     //=======================================================================
 
     int cspace_dim;
     if (sysType == 2)
         cspace_dim = 4;
     else if (sysType == 3)
         cspace_dim = 6;
     else
         OMPL_ERROR("Invalid system type. Must be 2 or 3");
 
     auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, cspace_dim));
 
 
     ob::RealVectorBounds bounds_ctrl(cspace_dim);
 
     if (dimension == 2)
     {
         bounds_ctrl.setLow(0, bounds_control[0][0]);
         bounds_ctrl.setHigh(0, bounds_control[0][1]);
         bounds_ctrl.setLow(1, bounds_control[0][0]);
         bounds_ctrl.setHigh(1, bounds_control[0][1]);
         bounds_ctrl.setLow(2, -M_PI);
         bounds_ctrl.setHigh(2, M_PI);
         bounds_ctrl.setLow(3, 0.05);
         bounds_ctrl.setHigh(3, 5.0);
     }
     else if (dimension == 3)
     {
         bounds_ctrl.setLow(0, bounds_control[0][0]);
         bounds_ctrl.setHigh(0, bounds_control[0][1]);
         bounds_ctrl.setLow(1, bounds_control[0][0]);
         bounds_ctrl.setHigh(1, bounds_control[0][1]);
         bounds_ctrl.setLow(2, bounds_control[0][0]);
         bounds_ctrl.setHigh(2, bounds_control[0][1]);
         bounds_ctrl.setLow(3, -M_PI);
         bounds_ctrl.setHigh(3, M_PI);
         bounds_ctrl.setLow(4, 0.05);
         bounds_ctrl.setHigh(4, 5.0);
         bounds_ctrl.setLow(5, -1.0); //heave
         bounds_ctrl.setHigh(5, 1.0);
     }
     else
         OMPL_ERROR("Invalid dimension. Must be 2 or 3");
 
     cspace->setBounds(bounds_ctrl);
     
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
 
     if (dimension == 2)
     {
         start[0] = double(initial_state[0]); //x
         start[1] = double(initial_state[1]); //y
         start[2] = double(0.0); // yaw
         start[3] = 1.0; //surge
     }
     else if (dimension == 3)
     {
         start[0] = double(initial_state[0]); //x
         start[1] = double(initial_state[1]); //y
         start[2] = double(initial_state[2]); //z
         start[3] = double(0.0); // yaw
         start[4] = 1.0; //surge
     }
     else
         OMPL_ERROR("Invalid dimension. Must be 2 or 3");
 
     simple_setup_->setStartState(start);
     double t_crit = quantile(boost::math::chi_squared(dimension), p_safe);
     simple_setup_->setGoal(std::make_shared<MyUnicycleGoalRegion>(si, p_safe, goal_state, goal_r, t_crit, dimension));
 
     if (dimension == 2)
         simple_setup_->setStatePropagator(oc::StatePropagatorPtr(new DynUnicycleControlSpaceFixedK(si, Q, R, R_bad, K, measurement_region))); //TODO: measurement
     else if (dimension == 3)
         simple_setup_->setStatePropagator(oc::StatePropagatorPtr(new DynUnicycleControlSpace3DFixedK(si, Q, R, R_bad, K, measurement_region)));
     else
         OMPL_ERROR("Invalid dimension. Must be 2 or 3");
 
     simple_setup_->getProblemDefinition()->setOptimizationObjective(getExpectedPathLengthObjective(si));
     ob::StateValidityCheckerPtr val_checker;
     val_checker = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore(scene, simple_setup_->getSpaceInformation(), p_safe, sysType));
     simple_setup_->setStateValidityChecker(val_checker);
 
     //=======================================================================
     // Perform setup steps for the planner
     //=======================================================================
     simple_setup_->setup();
     OMPL_INFORM("Full benchmarks starting");
     this->solve(plan_time, goal_bias, Q, R, R_bad, sampling_bias, selection_radius, pruning_radius, 1, file, false);
 }


void OfflinePlannerUncertainty::planWithSimpleSetup(int sysType, double plan_time, double dt, double p_safe, double Q, double R, double R_bad, double K, std::string scene,  std::vector<std::vector<double>> measurement_region, std::vector<std::vector<double>> bounds_state, std::vector<std::vector<double>> bounds_control, std::vector< double> goal_state, double goal_r, std::vector< double> initial_state, double goal_bias, double selection_radius, double pruning_radius, double sampling_bias, double control_duration_low, double control_duration_high, std::string file)
{
    //=======================================================================
    // Instantiate the state space
    //=======================================================================
    int dimension;
    if (sysType == 0)
        dimension = 2;
    else if (sysType == 1)
        dimension = 3;
    else
        OMPL_ERROR("Invalid system type. Must be 0 or 1");

    ob::StateSpacePtr space(constructStateSpace(dimension)); // [x y surge yaw covariance(16)] -> size = 20
    // set the bounds for the R^2 part of R2BeliefSpace();

    ob::RealVectorBounds bounds_se2(dimension);

    for (int i = 0; i < dimension; i++)
    {
        bounds_se2.setLow(i, bounds_state[i][0]);
        bounds_se2.setHigh(i, bounds_state[i][1]);
    }

    if (dimension == 2)
    {
        space->as<R2BeliefSpace>()->setBounds(bounds_se2);
    }
    else if (dimension == 3)
    {
        space->as<R3BeliefSpace>()->setBounds(bounds_se2);
    }
    //=======================================================================
    // Instantiate the control space
    //=======================================================================
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, dimension));

    ob::RealVectorBounds bounds(dimension);

    for (int i = 0; i < dimension; i++)
    {
        bounds.setLow(i, bounds_control[i][0]);
        bounds.setHigh(i, bounds_control[i][1]);
    }
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

    for (int i = 0; i < dimension; i++)
    {
        start[i] = double(initial_state[i]);
    }

    simple_setup_->setStartState(start);
    double t_crit = quantile(boost::math::chi_squared(dimension), p_safe); //TODO: check this
    simple_setup_->setGoal(std::make_shared<MyGoalRegion>(si, p_safe, goal_state, goal_r, t_crit, dimension));
    //=======================================================================
    // set the propagation routine for this space
    //=======================================================================

    if (dimension == 2)
        simple_setup_->setStatePropagator(oc::StatePropagatorPtr(new SimpleStatePropagatorFixedK(si, Q, R, R_bad, K, measurement_region)));
    else if (dimension == 3)
        simple_setup_->setStatePropagator(oc::StatePropagatorPtr(new ThreeDSimpleStatePropagatorFixedK(si, Q, R, R_bad, K, measurement_region)));

	simple_setup_->getProblemDefinition()->setOptimizationObjective(getExpectedPathLengthObjective(si));
    ob::StateValidityCheckerPtr val_checker;
    val_checker = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore(scene, simple_setup_->getSpaceInformation(), p_safe, sysType));
    simple_setup_->setStateValidityChecker(val_checker);

    //=======================================================================
    // Perform setup steps for the planner
    //=======================================================================
    simple_setup_->setup();
    OMPL_INFORM("Benchmarking starting");
    // this->solve(plan_time, goal_bias, Q, R, R_bad, sampling_bias, selection_radius, pruning_radius, 1, file, true);
    // OMPL_INFORM("Full benchmarks starting");
    this->solve(plan_time, goal_bias, Q, R, R_bad, sampling_bias, selection_radius, pruning_radius, 1, file, false);
}
 
 void OfflinePlannerUncertainty::planWithSimpleSetup(int sysType, double plan_time, double dt, double p_safe, double Q, double R, double R_bad, double K, std::string scene,  std::vector<std::vector<double>> measurement_region, std::vector<std::vector<double>> bounds_state, std::vector<std::vector<double>> bounds_control, std::vector< double> goal_state, double goal_r, std::vector< double> initial_state, double goal_bias, double selection_radius, double pruning_radius, double sampling_bias, double control_duration_low, double control_duration_high, std::string file, Eigen::MatrixXd A_matrix, Eigen::MatrixXd B_matrix)
 {
     //=======================================================================
     // Instantiate the state space
     //=======================================================================
     int dimension;
     if (sysType == 0)
         dimension = 2;
     else if (sysType == 1)
         dimension = 3;
     else if (sysType == 4) // Double integrator
         dimension = 4;
     else
         OMPL_ERROR("Invalid system type. Must be 0, 1, or 4");
 
     ob::StateSpacePtr space(constructStateSpace(dimension));
     
     // set the bounds for the R^2 part of R2BeliefSpace();
     ob::RealVectorBounds bounds_se2(dimension);
 
     for (int i = 0; i < dimension; i++)
     {
         bounds_se2.setLow(i, bounds_state[i][0]);
         bounds_se2.setHigh(i, bounds_state[i][1]);
     }
 
     if (dimension == 2)
     {
         space->as<R2BeliefSpace>()->setBounds(bounds_se2);
     }
     else if (dimension == 3)
     {
         space->as<R3BeliefSpace>()->setBounds(bounds_se2);
     }
     else if (dimension == 4) // Double integrator
     {
         space->as<RNBeliefSpace>()->setBounds(bounds_se2);
     }
     
     //=======================================================================
     // Instantiate the control space
     //=======================================================================
     int controlDim = (sysType == 4) ? 2 : dimension; // For double integrator, 2 controls (ax, ay)
     auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, controlDim));
 
     ob::RealVectorBounds bounds(controlDim);
 
     for (int i = 0; i < controlDim; i++)
     {
         bounds.setLow(i, bounds_control[i][0]);
         bounds.setHigh(i, bounds_control[i][1]);
     }
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
 
     for (int i = 0; i < dimension; i++)
     {
         start[i] = double(initial_state[i]);
     }
 
     simple_setup_->setStartState(start);
     double t_crit = quantile(boost::math::chi_squared(dimension), p_safe);
     simple_setup_->setGoal(std::make_shared<MyGoalRegion>(si, p_safe, goal_state, goal_r, t_crit, dimension));
     
     //=======================================================================
     // set the propagation routine for this space
     //=======================================================================
 
     if (dimension == 2)
     {
         if (A_matrix.rows() > 0 && B_matrix.rows() > 0) {
             // Use custom A and B matrices
             OMPL_INFORM("Using custom A and B matrices for 2D system");
             simple_setup_->setStatePropagator(oc::StatePropagatorPtr(
                 new LinearRNBeliefPropagator(si, A_matrix, B_matrix, Q, R, R_bad, K, measurement_region)));
         } else {
             // Use default propagator
             simple_setup_->setStatePropagator(oc::StatePropagatorPtr(
                 new SimpleStatePropagatorFixedK(si, Q, R, R_bad, K, measurement_region)));
         }
     }
     else if (dimension == 3)
     {
         if (A_matrix.rows() > 0 && B_matrix.rows() > 0) {
             // Use custom A and B matrices
             OMPL_INFORM("Using custom A and B matrices for 3D system");
             simple_setup_->setStatePropagator(oc::StatePropagatorPtr(
                 new LinearRNBeliefPropagator(si, A_matrix, B_matrix, Q, R, R_bad, K, measurement_region)));
         } else {
             // Use default propagator
             simple_setup_->setStatePropagator(oc::StatePropagatorPtr(
                 new ThreeDSimpleStatePropagatorFixedK(si, Q, R, R_bad, K, measurement_region)));
         }
     }
     else if (dimension == 4) // Double integrator
     {
         OMPL_INFORM("Using double integrator system with provided A and B matrices");
         simple_setup_->setStatePropagator(oc::StatePropagatorPtr(
             new LinearRNBeliefPropagator(si, A_matrix, B_matrix, Q, R, R_bad, K, measurement_region)));
     }
 
     simple_setup_->getProblemDefinition()->setOptimizationObjective(getExpectedPathLengthObjective(si));
     ob::StateValidityCheckerPtr val_checker;
     val_checker = ob::StateValidityCheckerPtr(new StateValidityCheckerPCCBlackmore(scene, simple_setup_->getSpaceInformation(), p_safe, sysType));
     simple_setup_->setStateValidityChecker(val_checker);
 
     //=======================================================================
     // Perform setup steps for the planner
     //=======================================================================
     simple_setup_->setup();
     OMPL_INFORM("Benchmarking starting");
     this->solve(plan_time, goal_bias, Q, R, R_bad, sampling_bias, selection_radius, pruning_radius, 1, file, false);
 }
 
 void OfflinePlannerUncertainty::solve(double plan_time, double goal_bias, double Q, double R, double R_bad, double sampling_bias, double selection_radius, double pruning_radius, int distance_function, std::string file, bool first_solution)
 {
     //=======================================================================
     // Set state validity checking for this space
     //=======================================================================
     ompl::tools::MyBenchmark b(*simple_setup_, "wasserstein");
     std::string resultsfile = "../results/first/fixedK/" + file + "/";
     if (first_solution)
     {
     simple_setup_->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(10000.0));
     b.addExperimentParameter("first_solution", "bool", "1");
     }
     else
     {
         simple_setup_->getProblemDefinition()->getOptimizationObjective()->setCostThreshold(ob::Cost(0.0));
         b.addExperimentParameter("first_solution", "bool", "0");
         resultsfile = "../results/final/fixedK/" + file + "/";
     }
     
     b.addExperimentParameter("Q_value", "double",std::to_string(Q));
     b.addExperimentParameter("R", "double",std::to_string(R));
     b.addExperimentParameter("R_bad_value", "double", std::to_string(R_bad));
 
     b.setDir(resultsfile);
     
     // // We add the planners to evaluate.
     double goal_bias_ = goal_bias;
     double sampling_bias_ = sampling_bias;
 
     b.addExperimentParameter("bias_value", "double", std::to_string(sampling_bias_));
     
     // SST
     double selection_radius_ = selection_radius;
     double pruning_radius_ = pruning_radius;
     
     ob::PlannerPtr planner;
     planner = ob::PlannerPtr(new oc::SSBT(simple_setup_->getSpaceInformation()));
     planner->as<oc::SSBT>()->setGoalBias(goal_bias_);
     planner->as<oc::SSBT>()->setSelectionRadius(selection_radius_);
     planner->as<oc::SSBT>()->setPruningRadius(pruning_radius_);
     planner->as<oc::SSBT>()->setSamplingBias(sampling_bias_);
     planner->as<oc::SSBT>()->setDistanceFunction(distance_function); //wasserstein
     b.addPlanner(planner);
 
     ob::PlannerPtr planner_rrt;
     planner_rrt = ob::PlannerPtr(new oc::mod_RRT(simple_setup_->getSpaceInformation()));
     planner_rrt->as<oc::mod_RRT>()->setGoalBias(goal_bias_);
     planner_rrt->as<oc::mod_RRT>()->setSamplingBias(sampling_bias_);
     planner_rrt->as<oc::mod_RRT>()->setDistanceFunction(distance_function); //wasserstein
     b.addPlanner(planner_rrt);
     
     OMPL_INFORM("Benchmarking");
 
     ompl::tools::MyBenchmark::MyRequest req;
     req.maxTime = plan_time;
     req.maxMem = 1000.0;
     req.runCount = 100;
     req.displayProgress = true;
     b.benchmark(req);
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
     double Q = std::stod(pt.get<std::string>("System.Q"));
     double R = std::stod(pt.get<std::string>("System.R"));
     double R_bad = std::stod(pt.get<std::string>("System.R_bad"));
     double K = std::stod(pt.get<std::string>("System.K"));
     double dt = std::stod(pt.get<std::string>("System.dt"));
     std::string scene = pt.get<std::string>("Scene.scene");
 
     scene = "../scenes/" + scene + ".yaml";
 
     // A and B matrices for custom dynamics
     Eigen::MatrixXd A_matrix, B_matrix;
     bool useCustomDynamics = false;
     
     // Check if A and B matrices are defined in the config
     if (pt.get_optional<std::string>("System.A_matrix").is_initialized() && 
         pt.get_optional<std::string>("System.B_matrix").is_initialized()) {
         
         std::string A_str = pt.get<std::string>("System.A_matrix");
         std::string B_str = pt.get<std::string>("System.B_matrix");
         
         // Determine matrix dimensions based on system type
         int stateDim = 2; // Default for sysType 0
         int controlDim = 2; // Default
         
         if (sysType == 1) {
             stateDim = 3;
             controlDim = 3;
         } else if (sysType == 4) { // Double integrator
             stateDim = 4;
             controlDim = 2;
         }
         
         // Parse matrices
         A_matrix = parseMatrix(A_str, stateDim, stateDim);
         B_matrix = parseMatrix(B_str, stateDim, controlDim);
         
         useCustomDynamics = true;
         OMPL_INFORM("Using custom dynamics matrices:");
         std::cout << "A matrix:" << std::endl << A_matrix << std::endl;
         std::cout << "B matrix:" << std::endl << B_matrix << std::endl;
     }
 
     // Planner params
     double p_safe = std::stod(pt.get<std::string>("Planner.p_safe"));
     double plan_time = std::stod(pt.get<std::string>("Planner.planning_time"));
     std::string goal = pt.get<std::string>("Planner.goal");
     std::vector<std::string> spltStr = split(goal, ",");
     
     // Parse goal state based on system type
     std::vector<double> goal_state;
     for (size_t i = 0; i < spltStr.size(); i++) {
         goal_state.push_back(std::stod(spltStr[i]));
     }
     
     // If we're using a double integrator, ensure goal state has velocity components
     if (sysType == 4 && goal_state.size() == 2) {
         // Add zero velocity goals if not specified
         goal_state.push_back(0.0); // vx = 0
         goal_state.push_back(0.0); // vy = 0
     }
     
     double goal_r = std::stod(pt.get<std::string>("Planner.goal_radius"));
     std::string init = pt.get<std::string>("Planner.initial_state");
     spltStr = split(init, ",");
     
     // Parse initial state based on system type
     std::vector<double> initial_state;
     for (size_t i = 0; i < spltStr.size(); i++) {
         initial_state.push_back(std::stod(spltStr[i]));
     }
     
     // If we're using a double integrator, ensure initial state has velocity components
     if (sysType == 4 && initial_state.size() == 2) {
         // Add zero initial velocities if not specified
         initial_state.push_back(0.0); // vx = 0
         initial_state.push_back(0.0); // vy = 0
     }
 
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
     std::vector<std::vector<double>> bounds_state;
     for(; iterBound < Boundsplt.end(); iterBound++)
     {
         std::vector<std::string> spltStrBound_rows = split(*iterBound, ","); // Split rows
         std::vector<double> rowsBound_doub(spltStrBound_rows.size());
         std::transform(spltStrBound_rows.begin(), spltStrBound_rows.end(), rowsBound_doub.begin(), [](const std::string& val)
             {
                 return std::stod(val);
             });
         bounds_state.push_back(rowsBound_doub);
     }
 
     // If using double integrator (sysType 4) and state bounds don't include velocity bounds,
     // add default velocity bounds
     if (sysType == 4 && bounds_state.size() == 2) {
         // Add default velocity bounds if not provided
         bounds_state.push_back({-5.0, 5.0}); // vx bounds
         bounds_state.push_back({-5.0, 5.0}); // vy bounds
     }
 
     // Bounds on controls
     Boundstr = pt.get<std::string>("Environment.control_bounds");
     Boundsplt = split(Boundstr, ":"); // Split rows
     iterBound = Boundsplt.begin(); // Iterate through and build vector of doubles
     std::vector<std::vector<double>> bounds_control;
     for(; iterBound < Boundsplt.end(); iterBound++)
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
     std::vector<std::vector<double>> measurement_region;
     for(; iterBound < Boundsplt.end(); iterBound++)
     {
         std::vector<std::string> spltStrBound_rows = split(*iterBound, ","); // Split rows
         std::vector<double> rowsBound_doub(spltStrBound_rows.size());
         std::transform(spltStrBound_rows.begin(), spltStrBound_rows.end(), rowsBound_doub.begin(), [](const std::string& val)
             {
                 return std::stod(val);
             });
         measurement_region.push_back(rowsBound_doub);
     }
 
     // If using double integrator and measurement regions don't include velocity dimensions,
     // add default measurement regions for velocities
     if (sysType == 4 && measurement_region.size() == 2) {
         // Add default measurement regions for velocity dimensions
         measurement_region.push_back({-5.0, 5.0}); // vx measurement region
         measurement_region.push_back({-5.0, 5.0}); // vy measurement region
     }
 
     OMPL_INFORM("USING P_SAFE: %3f, Q: %3f, R: %3f", p_safe, Q, R);
 
     OfflinePlannerUncertainty offline_planner_uncertainty;
 
     if (sysType == 1) {
         OMPL_INFORM("Using 3d linear system");
         if (useCustomDynamics) {
             offline_planner_uncertainty.planWithSimpleSetup(sysType, plan_time, dt, p_safe, Q, R, R_bad, K, 
                 scene, measurement_region, bounds_state, bounds_control, goal_state, goal_r, initial_state, 
                 goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, 
                 control_duration_high, file, A_matrix, B_matrix);
         } else {
             offline_planner_uncertainty.planWithSimpleSetup(sysType, plan_time, dt, p_safe, Q, R, R_bad, K, 
                 scene, measurement_region, bounds_state, bounds_control, goal_state, goal_r, initial_state, 
                 goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, 
                 control_duration_high, file);
         }
     } else if (sysType == 0) {
         OMPL_INFORM("Using linear system");
         if (useCustomDynamics) {
             offline_planner_uncertainty.planWithSimpleSetup(sysType, plan_time, dt, p_safe, Q, R, R_bad, K, 
                 scene, measurement_region, bounds_state, bounds_control, goal_state, goal_r, initial_state, 
                 goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, 
                 control_duration_high, file, A_matrix, B_matrix);
         } else {
             offline_planner_uncertainty.planWithSimpleSetup(sysType, plan_time, dt, p_safe, Q, R, R_bad, K, 
                 scene, measurement_region, bounds_state, bounds_control, goal_state, goal_r, initial_state, 
                 goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, 
                 control_duration_high, file);
         }
     } else if (sysType == 2) {
         OMPL_INFORM("Using linearized unicycle system");
         // Bounds on surge
         std::string Boundstr = pt.get<std::string>("Environment.bounds_surge");
         std::vector<std::string> Boundsplt = split(Boundstr, ":"); // Split rows
         std::vector<std::string>::iterator iterBound = Boundsplt.begin(); // Iterate through and build vector of doubles
         std::vector<std::vector<double>> bounds_surge;
         for(; iterBound < Boundsplt.end(); iterBound++)
         {
             std::vector<std::string> spltStrBound_rows = split(*iterBound, ","); // Split rows
             std::vector<double> rowsBound_doub(spltStrBound_rows.size());
             std::transform(spltStrBound_rows.begin(), spltStrBound_rows.end(), rowsBound_doub.begin(), [](const std::string& val)
                 {
                     return std::stod(val);
                 });
             bounds_surge.push_back(rowsBound_doub);
         }
         offline_planner_uncertainty.planWithUnicycle(sysType, plan_time, dt, p_safe, Q, R, R_bad, K, 
             scene, measurement_region, bounds_state, bounds_surge, bounds_control, goal_state, goal_r, 
             initial_state, goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, 
             control_duration_high, file);
     } else if (sysType == 4) {
         OMPL_INFORM("Using double integrator system");
         if (!useCustomDynamics) {
             // Create default double integrator A and B matrices if not provided
             // For 2D double integrator with state [x, y, vx, vy]:
             //   A = [0 0 1 0]
             //       [0 0 0 1]
             //       [0 0 0 0]
             //       [0 0 0 0]
             //
             //   B = [0 0]
             //       [0 0]
             //       [1 0]
             //       [0 1]
             
             A_matrix = Eigen::MatrixXd::Identity(4, 4);
             A_matrix(0, 2) = dt; // x' = vx
             A_matrix(1, 3) = dt; // y' = vy
             
             B_matrix = Eigen::MatrixXd::Zero(4, 2);
             B_matrix(0,0) = 0.5*dt*dt;
             B_matrix(1,1) = 0.5*dt*dt;
             B_matrix(2, 0) = dt; // vx' = ax
             B_matrix(3, 1) = dt; // vy' = ay
             
             OMPL_INFORM("Using default double integrator dynamics");
             std::cout << "A matrix:" << std::endl << A_matrix << std::endl;
             std::cout << "B matrix:" << std::endl << B_matrix << std::endl;
         }
         
         offline_planner_uncertainty.planWithSimpleSetup(sysType, plan_time, dt, p_safe, Q, R, R_bad, K, 
             scene, measurement_region, bounds_state, bounds_control, goal_state, goal_r, initial_state, 
             goal_bias, selection_radius, pruning_radius, sampling_bias, control_duration_low, 
             control_duration_high, file, A_matrix, B_matrix);
     } else {
         OMPL_ERROR("Invalid system type");
     }    
     
     return 0;
 }