#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RNBeliefSpace.h"
#include <ompl/control/Control.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <boost/math/special_functions/erf.hpp>

namespace ompl {
namespace base {

//--------------------------------------------------
// Belief struct (copied from BarrierValidityChecker.cpp)
//--------------------------------------------------
struct Belief {
    Eigen::VectorXd mu;       // Mean
    Eigen::MatrixXd Sigma;    // Covariance
    Eigen::MatrixXd Lambda;   // Additional uncertainty
};

//--------------------------------------------------
// Belief derivative struct (copied from BarrierValidityChecker.cpp)
//--------------------------------------------------
struct BeliefDerivative {
    Eigen::VectorXd dmu;
    Eigen::MatrixXd dSigma;
    Eigen::MatrixXd dLambda;
};

//--------------------------------------------------
// Risk-aware half-space result struct (copied from BarrierValidityChecker.cpp)
//--------------------------------------------------
struct RiskAwareResult {
    double h;
    Eigen::VectorXd dmu;
    Eigen::MatrixXd dSigma;
    Eigen::MatrixXd dLambda;
};

BarrierTrajectoryValidityChecker::BarrierTrajectoryValidityChecker(const SpaceInformationPtr &si)
    : StateValidityChecker(si), delta_(0.01), N_(10), dt_(0.1)
{
    // Initialize system matrices with default values
    A_ = Eigen::MatrixXd::Identity(2, 2);
    B_ = Eigen::MatrixXd::Identity(2, 2);
    K_ = Eigen::MatrixXd::Zero(2, 2);
    G_ = Eigen::MatrixXd::Identity(2, 2);
    Q_ = Eigen::MatrixXd::Identity(2, 2);
}

bool BarrierTrajectoryValidityChecker::isValid(const State *state) const
{
    // For single state validation, we can still use the existing logic
    // or delegate to a simpler checker if needed
    return true; // Default implementation - override as needed
}

bool BarrierTrajectoryValidityChecker::isValidTrajectory(const State *initial_state,
                                                        const std::vector<ompl::control::Control*> &controls,
                                                        const std::vector<double> &durations) const
{
    return checkTrajectoryBarrierConstraints(initial_state, controls, durations);
}

void BarrierTrajectoryValidityChecker::setSystemMatrices(const Eigen::MatrixXd &A,
                                                        const Eigen::MatrixXd &B,
                                                        const Eigen::MatrixXd &K,
                                                        const Eigen::MatrixXd &G,
                                                        const Eigen::MatrixXd &Q)
{
    A_ = Eigen::MatrixXd::Zero(2, 2);
    B_ = B;
    K_ = 0.8*Eigen::MatrixXd::Identity(2,2);;
    G_ = G;
    Q_ = Q;
}

void BarrierTrajectoryValidityChecker::setHalfSpaceConstraints(const std::vector<Eigen::VectorXd> &a_list,
                                                              const std::vector<double> &gamma_list,
                                                              double delta)
{
    a_list_ = a_list;
    gamma_list_ = gamma_list;
    delta_ = delta;
}

void BarrierTrajectoryValidityChecker::setTimeParameters(int N, double dt)
{
    N_ = N;
    dt_ = dt;
}

//--------------------------------------------------
// Propagate belief given control u (copied from BarrierValidityChecker.cpp)
//--------------------------------------------------
static BeliefDerivative propagateBelief(
    const Belief &b,
    const Eigen::VectorXd &u,
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &K,
    const Eigen::MatrixXd &G,
    const Eigen::MatrixXd &Q)
{
    BeliefDerivative b_dot;
    b_dot.dmu = A * b.mu + B * u;
    b_dot.dSigma = A * b.Sigma + b.Sigma * A.transpose() + G * Q * G.transpose();
    Eigen::MatrixXd A_minus_BK = A - B * K;
    b_dot.dLambda = A_minus_BK * b.Lambda + b.Lambda * A_minus_BK.transpose();
    return b_dot;
}

//--------------------------------------------------
// Risk-aware half-space and gradient (copied from BarrierValidityChecker.cpp)
//--------------------------------------------------
static RiskAwareResult riskAwareHalfspaceWithGradient(
    const Belief &b,
    const Eigen::VectorXd &a,
    const double gamma,
    const double delta)
{
    RiskAwareResult result;
    Eigen::MatrixXd totalCov = b.Sigma + b.Lambda;
    double aSigmaA = a.transpose() * totalCov * a;
    double erfinv_result = boost::math::erf_inv(1 - 2*delta);
    double sqrt2aSigmaA = std::sqrt(2.0 * aSigmaA);

    // Risk-aware half-space value
    result.h = a.transpose() * b.mu - gamma - sqrt2aSigmaA * erfinv_result;

    // Gradients
    result.dmu = a;
    Eigen::MatrixXd gradCov = -erfinv_result * (a * a.transpose()) / std::sqrt(aSigmaA);
    result.dSigma = gradCov;
    result.dLambda = gradCov;

    // std::cout << "        Risk-aware calculation:" << std::endl;
    // std::cout << "          a^T * mu: " << (a.transpose() * b.mu)(0) << std::endl;
    // std::cout << "          gamma: " << gamma << std::endl;
    // std::cout << "          sqrt(2 * a^T * Sigma * a): " << sqrt2aSigmaA << std::endl;
    // std::cout << "          erf_inv(1-2*delta): " << erfinv_result << std::endl;
    // std::cout << "          h = " << result.h << std::endl;

    return result;
}

//--------------------------------------------------
// Barrier check for multiple half-space constraints (copied from BarrierValidityChecker.cpp)
//--------------------------------------------------
static bool barrierCheckMultiple(
    const Belief &b,
    const std::vector<Eigen::VectorXd> &a_list,
    const std::vector<double> &gamma_list,
    double delta,
    const BeliefDerivative &b_dot,
    const Eigen::MatrixXd &B,
    const Eigen::VectorXd &u)
{
    // std::cout << "    Checking " << a_list.size() << " half-space constraints..." << std::endl;
    
    // Iterate over all half-space constraints
    for(size_t l = 0; l < a_list.size(); ++l)
    {
        RiskAwareResult res = riskAwareHalfspaceWithGradient(b, a_list[l], gamma_list[l], delta);

        // std::cout << "      Constraint " << l << ":" << std::endl;
        // std::cout << "        a: [" << a_list[l].transpose() << "]" << std::endl;
        // std::cout << "        gamma: " << gamma_list[l] << std::endl;
        // std::cout << "        h: " << res.h << std::endl;

        // Evaluate the risk-aware barrier condition
        double lhs = res.dmu.dot(b_dot.dmu)
                     + (res.dSigma.array() * b_dot.dSigma.array()).sum()
                     + (res.dLambda.array() * b_dot.dLambda.array()).sum()
                     + res.dmu.dot(B * u);

        // std::cout << "        lhs: " << lhs << ", -h: " << -res.h << std::endl;
        // std::cout << "        satisfied: " << (lhs >= -res.h ? "YES" : "NO") << std::endl;

        // If any one constraint holds, return true
        if(lhs >= -res.h)
        {
            // std::cout << "        ✅ Constraint " << l << " satisfied" << std::endl;
            return true;
        }
    }

    // std::cout << "        ❌ All constraints violated" << std::endl;
    // None of the constraints satisfied → violated
    return false;
}

bool BarrierTrajectoryValidityChecker::checkTrajectoryBarrierConstraints(
    const State *initial_state,
    const std::vector<ompl::control::Control*> &controls,
    const std::vector<double> &durations) const
{
    if (a_list_.empty() || controls.empty())
    {
        // std::cout << "BarrierTrajectoryValidityChecker: No constraints or controls to check" << std::endl;
        return true; // No constraints to check
    }

    // std::cout << "BarrierTrajectoryValidityChecker: Checking trajectory with " 
    //           << controls.size() << " controls and " << a_list_.size() << " constraints" << std::endl;

    // Extract initial belief state
    Belief b;
    extractBeliefState(initial_state, b.mu, b.Sigma, b.Lambda);
    
    // std::cout << "Initial belief state:" << std::endl;
    // std::cout << "  mu: [" << b.mu.transpose() << "]" << std::endl;
    // std::cout << "  Sigma:\n" << b.Sigma << std::endl;
    // std::cout << "  Lambda:\n" << b.Lambda << std::endl;
    
    // Check constraints at each control step
    for (size_t i = 0; i < controls.size(); ++i)
    {
        // Extract control
        Eigen::VectorXd u;
        extractControl(controls[i], u);
        
        // std::cout << "Control " << i << ": [" << u.transpose() << "]" << std::endl;
        
        // Calculate number of time steps for this control duration
        int num_steps = static_cast<int>(durations[i] / dt_);
        // std::cout << "  Duration: " << durations[i] << ", Steps: " << num_steps << std::endl;
        
        // Propagate belief and check constraints at each time step
        for (int step = 0; step <= num_steps; ++step)
        {
            // std::cout << "  Step " << step << "/" << num_steps << std::endl;
            
            BeliefDerivative b_dot = propagateBelief(b, u, A_, B_, K_, G_, Q_);
            
            // std::cout << "    Belief derivative:" << std::endl;
            // std::cout << "      dmu: [" << b_dot.dmu.transpose() << "]" << std::endl;
            // std::cout << "      dSigma:\n" << b_dot.dSigma << std::endl;
            // std::cout << "      dLambda:\n" << b_dot.dLambda << std::endl;
            
            // Check all half-space constraints in a risk-aware manner
            bool constraint_satisfied = barrierCheckMultiple(b, a_list_, gamma_list_, delta_, b_dot, B_, u);
            // std::cout << "    Constraint satisfied: " << (constraint_satisfied ? "YES" : "NO") << std::endl;
            
            if (!constraint_satisfied)
            {
                // std::cout << "    ❌ CONSTRAINT VIOLATED at step " << step << std::endl;
                return false; // violated
            }
            
            // Euler integration to propagate belief (only if not the last step)
            if (step < num_steps)
            {
                b.mu += dt_ * b_dot.dmu;
                b.Sigma += dt_ * b_dot.dSigma;
                b.Lambda += dt_ * b_dot.dLambda;
                
                // std::cout << "    Updated belief state:" << std::endl;
                // std::cout << "      mu: [" << b.mu.transpose() << "]" << std::endl;
                // std::cout << "      Sigma trace: " << b.Sigma.trace() << std::endl;
            }
        }
    }
    
    // std::cout << "✅ All constraints satisfied for entire trajectory" << std::endl;
    return true; // satisfied for all steps
}

void BarrierTrajectoryValidityChecker::extractBeliefState(const State *state,
                                                         Eigen::VectorXd &mu,
                                                         Eigen::MatrixXd &Sigma,
                                                         Eigen::MatrixXd &Lambda) const
{
    // std::cout << "Extracting belief state from state type..." << std::endl;
    
    // Try to extract from R2BeliefSpace first
    if (auto belief_state = dynamic_cast<const R2BeliefSpace::StateType*>(state))
    {
        // std::cout << "  Using R2BeliefSpace" << std::endl;
        mu.resize(2);
        mu << belief_state->getX(), belief_state->getY();
        Sigma = belief_state->getCovariance();
        Lambda = Eigen::MatrixXd::Zero(2, 2); // Default value
    }
    // Try to extract from RNBeliefSpace
    else if (auto belief_state = dynamic_cast<const RNBeliefSpace::StateType*>(state))
    {
        int dim = belief_state->getDimension();
        // std::cout << "  Using RNBeliefSpace with dimension " << dim << std::endl;
        mu.resize(dim);
        for (int i = 0; i < dim; ++i)
            mu(i) = belief_state->getComponent(i);
        Sigma = belief_state->getCovariance();
        Lambda = Eigen::MatrixXd::Zero(dim, dim); // Default value
        
        // std::cout << "  Extracted mu: [" << mu.transpose() << "]" << std::endl;
        // std::cout << "  Extracted Sigma:\n" << Sigma << std::endl;
    }
    // Try to extract from compound state space
    else if (auto compound_state = dynamic_cast<const ompl::base::CompoundStateSpace::StateType*>(state))
    {
        // std::cout << "  Using CompoundStateSpace" << std::endl;
        // Try to get the first component as a belief state
        if (auto belief_state = dynamic_cast<const R2BeliefSpace::StateType*>(compound_state->components[0]))
        {
            // std::cout << "    First component is R2BeliefSpace" << std::endl;
            mu.resize(2);
            mu << belief_state->getX(), belief_state->getY();
            Sigma = belief_state->getCovariance();
            Lambda = Eigen::MatrixXd::Zero(2, 2);
        }
        else if (auto belief_state = dynamic_cast<const RNBeliefSpace::StateType*>(compound_state->components[0]))
        {
            int dim = belief_state->getDimension();
            // std::cout << "    First component is RNBeliefSpace with dimension " << dim << std::endl;
            mu.resize(dim);
            for (int i = 0; i < dim; ++i)
                mu(i) = belief_state->getComponent(i);
            Sigma = belief_state->getCovariance();
            Lambda = Eigen::MatrixXd::Zero(dim, dim);
        }
        else
        {
            // std::cout << "    First component is not a belief state, using fallback" << std::endl;
            // Fallback: create default values
            mu = Eigen::VectorXd::Zero(2);
            Sigma = Eigen::MatrixXd::Identity(2, 2);
            Lambda = Eigen::MatrixXd::Zero(2, 2);
        }
    }
    else
    {
        // std::cout << "  Unknown state type, using fallback" << std::endl;
        // Fallback: create default values
        mu = Eigen::VectorXd::Zero(2);
        Sigma = Eigen::MatrixXd::Identity(2, 2);
        Lambda = Eigen::MatrixXd::Zero(2, 2);
    }
}

void BarrierTrajectoryValidityChecker::extractControl(const ompl::control::Control *control, Eigen::VectorXd &u) const
{
    // std::cout << "Extracting control..." << std::endl;
    
    if (auto real_vector_control = dynamic_cast<const ompl::control::RealVectorControlSpace::ControlType*>(control))
    {
        // Control dimension is 2 (x_vel, y_vel) - duration is handled by OMPL separately
        int control_dim = 2;
        // std::cout << "  RealVectorControlSpace with dimension " << control_dim << std::endl;
        
        u.resize(control_dim);
        
        // Extract all control components
        for (int i = 0; i < control_dim; ++i)
        {
            u(i) = real_vector_control->values[i];
        }
        
        // std::cout << "  Extracted control: [" << u.transpose() << "]" << std::endl;
    }
    else
    {
        // std::cout << "  Unknown control type, using fallback" << std::endl;
        // Fallback: create default control (2D for belief space)
        u = Eigen::VectorXd::Zero(2);
    }
}

} // namespace base
} // namespace ompl 