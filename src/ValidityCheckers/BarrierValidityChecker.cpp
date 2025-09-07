#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "Spaces/R2BeliefSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/RealVectorControlSpace.h>

using namespace ompl;
using namespace ompl::base;
using namespace ompl::control;

int main()
{
    std::cout << "Barrier Trajectory Validity Checker Example" << std::endl;
    
    // 1. Create state space
    auto state_space = std::make_shared<R2BeliefSpace>(5.0);
    
    // 2. Create control space
    auto control_space = std::make_shared<RealVectorControlSpace>(state_space, 3);
    
    // 3. Create space information
    auto si = std::make_shared<SpaceInformation>(state_space, control_space);
    
    // 4. Create the trajectory validity checker
    auto validity_checker = std::make_shared<BarrierTrajectoryValidityChecker>(si);
    
    // 5. Set system matrices (example for a simple 2D system)
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd Q = 0.1 * Eigen::MatrixXd::Identity(2, 2);
    
    validity_checker->setSystemMatrices(A, B, K, G, Q);
    
    // 6. Set half-space constraints (example: stay within bounds)
    std::vector<Eigen::VectorXd> a_list;
    std::vector<double> gamma_list;
    
    // Constraint: x >= 0 (left boundary)
    Eigen::VectorXd a1(2);
    a1 << -1.0, 0.0;
    a_list.push_back(a1);
    gamma_list.push_back(0.0);
    
    // Constraint: x <= 100 (right boundary)
    Eigen::VectorXd a2(2);
    a2 << 1.0, 0.0;
    a_list.push_back(a2);
    gamma_list.push_back(100.0);
    
    // Constraint: y >= 0 (bottom boundary)
    Eigen::VectorXd a3(2);
    a3 << 0.0, -1.0;
    a_list.push_back(a3);
    gamma_list.push_back(0.0);
    
    // Constraint: y <= 100 (top boundary)
    Eigen::VectorXd a4(2);
    a4 << 0.0, 1.0;
    a_list.push_back(a4);
    gamma_list.push_back(100.0);
    
    validity_checker->setHalfSpaceConstraints(a_list, gamma_list, 0.01); // 1% risk
    
    // 7. Set time parameters
    validity_checker->setTimeParameters(10, 0.1); // 10 steps, 0.1s each
    
    // 8. Create a test initial state
    auto initial_state = state_space->allocState();
    auto belief_state = initial_state->as<R2BeliefSpace::StateType>();
    belief_state->setX(10.0);
    belief_state->setY(10.0);
    belief_state->setSigma(0.5 * Eigen::MatrixXd::Identity(2, 2));
    
    // 9. Create test controls
    std::vector<Control*> controls;
    std::vector<double> durations;
    
    // Control 1: move right
    auto control1 = control_space->allocControl();
    auto real_control1 = control1->as<RealVectorControlSpace::ControlType>();
    real_control1->values[0] = 10.0;  // x velocity
    real_control1->values[1] = 0.0;   // y velocity
    real_control1->values[2] = 0.1;   // time duration
    controls.push_back(control1);
    durations.push_back(1.0);
    
    // Control 2: move up
    auto control2 = control_space->allocControl();
    auto real_control2 = control2->as<RealVectorControlSpace::ControlType>();
    real_control2->values[0] = 0.0;   // x velocity
    real_control2->values[1] = 10.0;  // y velocity
    real_control2->values[2] = 0.1;   // time duration
    controls.push_back(control2);
    durations.push_back(1.0);
    
    // 10. Test trajectory validity
    bool is_valid = validity_checker->isValidTrajectory(initial_state, controls, durations);
    
    std::cout << "Trajectory validity: " << (is_valid ? "VALID" : "INVALID") << std::endl;
    
    // 11. Clean up
    for (auto control : controls) {
        control_space->freeControl(control);
    }
    state_space->freeState(initial_state);
    
    return 0;
}
