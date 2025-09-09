#ifndef BARRIER_TRAJECTORY_VALIDITY_CHECKER_HPP
#define BARRIER_TRAJECTORY_VALIDITY_CHECKER_HPP

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <Eigen/Dense>
#include <vector>

namespace ompl {
namespace base {

class BarrierTrajectoryValidityChecker : public StateValidityChecker
{
public:
    BarrierTrajectoryValidityChecker(const SpaceInformationPtr &si);
    
    virtual bool isValid(const State *state) const override;
    
    // New method to check trajectory validity
    bool isValidTrajectory(const State *initial_state, 
                          const std::vector<ompl::control::Control*> &controls,
                          const std::vector<double> &durations) const;
    
    // Set system matrices for belief propagation
    void setSystemMatrices(const Eigen::MatrixXd &A, 
                          const Eigen::MatrixXd &B,
                          const Eigen::MatrixXd &K,
                          const Eigen::MatrixXd &G,
                          const Eigen::MatrixXd &Q);
    
    // Set half-space constraints for a single obstacle
    void setHalfSpaceConstraints(const std::vector<Eigen::VectorXd> &a_list,
                                const std::vector<double> &gamma_list,
                                double delta);
    
    // Set multiple obstacles, each with their own half-space constraints
    void setMultipleObstacles(const std::vector<std::vector<Eigen::VectorXd>> &obstacle_a_lists,
                             const std::vector<std::vector<double>> &obstacle_gamma_lists,
                             double delta);
    
    // Set time discretization parameters
    void setTimeParameters(int N, double dt);

    void setNumConstraints(int n);

private:
    // System matrices for belief propagation
    Eigen::MatrixXd A_, B_, K_, G_, Q_;
    
    // Half-space constraints (for single obstacle mode)
    std::vector<Eigen::VectorXd> a_list_;
    std::vector<double> gamma_list_;
    
    // Multiple obstacles (each obstacle has its own set of half-space constraints)
    std::vector<std::vector<Eigen::VectorXd>> obstacle_a_lists_;
    std::vector<std::vector<double>> obstacle_gamma_lists_;
    
    double delta_;

     int numConstraints_;
    
    // Time discretization
    int N_;
    double dt_;
    
    // Belief propagation and barrier checking methods
    bool checkTrajectoryBarrierConstraints(const State *initial_state,
                                         const std::vector<ompl::control::Control*> &controls,
                                         const std::vector<double> &durations) const;
    
    // Helper method to extract belief state from OMPL state
    void extractBeliefState(const State *state, 
                           Eigen::VectorXd &mu, 
                           Eigen::MatrixXd &Sigma, 
                           Eigen::MatrixXd &Lambda) const;
    
    // Helper method to extract control from OMPL control
    void extractControl(const ompl::control::Control *control, Eigen::VectorXd &u) const;
};

} // namespace base
} // namespace ompl

#endif // BARRIER_TRAJECTORY_VALIDITY_CHECKER_HPP 