#include "ValidityCheckers/BarrierTrajectoryValidityChecker.hpp"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RNBeliefSpace.h"

#include <ompl/control/Control.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <boost/math/special_functions/erf.hpp>

#include <random>
#include <cmath>
#include <iostream>

namespace ompl {
namespace base {

//--------------------------------------------------
// Belief struct (kept local to this TU)
//--------------------------------------------------
struct Belief {
    Eigen::VectorXd mu;       // Mean
    Eigen::MatrixXd Sigma;    // Covariance
    Eigen::MatrixXd Lambda;   // Additional uncertainty
};

//--------------------------------------------------
// Belief derivative struct (kept local to this TU)
//--------------------------------------------------
struct BeliefDerivative {
    Eigen::VectorXd dmu;
    Eigen::MatrixXd dSigma;
    Eigen::MatrixXd dLambda;
};

//--------------------------------------------------
// Risk-aware half-space result struct (kept local to this TU)
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

bool BarrierTrajectoryValidityChecker::isValid(const State * /*state*/) const
{
    // Default implementation - override as needed
    return true;
}

bool BarrierTrajectoryValidityChecker::isValidTrajectory(
    const State *initial_state,
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
    // Preserve prior behavior (A hardcoded to zeros / K to 0.8 I as in your base)
    A_ = Eigen::MatrixXd::Zero(2, 2);
    B_ = B;
    K_ = 0.8 * Eigen::MatrixXd::Identity(2, 2);
    G_ = G;
    Q_ = Q;
}

void BarrierTrajectoryValidityChecker::setHalfSpaceConstraints(const std::vector<Eigen::VectorXd> &a_list,
                                                               const std::vector<double> &gamma_list,
                                                               double delta)
{
    a_list_      = a_list;
    gamma_list_  = gamma_list;
    delta_       = delta;

    // Clear multiple obstacles when using single obstacle mode
    obstacle_a_lists_.clear();
    obstacle_gamma_lists_.clear();
}

void BarrierTrajectoryValidityChecker::setMultipleObstacles(
    const std::vector<std::vector<Eigen::VectorXd>> &obstacle_a_lists,
    const std::vector<std::vector<double>> &obstacle_gamma_lists,
    double delta)
{
    std::cout << "setMultipleObstacles called with " << obstacle_a_lists.size() << " obstacles" << std::endl;
    for (size_t i = 0; i < obstacle_a_lists.size(); ++i) {
        std::cout << "  Obstacle " << i << ": " << obstacle_a_lists[i].size() << " constraints" << std::endl;
    }

    obstacle_a_lists_   = obstacle_a_lists;
    obstacle_gamma_lists_ = obstacle_gamma_lists;
    delta_ = delta;

    // Clear single obstacle mode when using multiple obstacles
    a_list_.clear();
    gamma_list_.clear();

    std::cout << "Multiple obstacles setup complete" << std::endl;
}

void BarrierTrajectoryValidityChecker::setTimeParameters(int N, double dt)
{
    N_  = N;
    dt_ = dt;
}

//--------------------------------------------------
// Propagate belief given control u
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
    b_dot.dmu    = A * b.mu + B * u;
    b_dot.dSigma = A * b.Sigma + b.Sigma * A.transpose() + G * Q * G.transpose();
    Eigen::MatrixXd A_minus_BK = A - B * K;
    b_dot.dLambda = A_minus_BK * b.Lambda + b.Lambda * A_minus_BK.transpose();
    return b_dot;
}

//--------------------------------------------------
// Risk-aware half-space and gradient
//--------------------------------------------------
static RiskAwareResult riskAwareHalfspaceWithGradient(
    const Belief &b,
    const Eigen::VectorXd &a,
    const double gamma,
    const double delta)
{
    RiskAwareResult result;
    Eigen::MatrixXd totalCov = b.Sigma + b.Lambda;
    double aSigmaA = (a.transpose() * totalCov * a)(0, 0);
    double erfinv_result = boost::math::erf_inv(1 - 2 * delta);
    double sqrt2aSigmaA = std::sqrt(2.0 * std::max(aSigmaA, 0.0));

    // Risk-aware half-space value
    result.h = a.transpose() * b.mu - gamma - sqrt2aSigmaA * erfinv_result;

    // Gradients
    result.dmu = a;
    Eigen::MatrixXd gradCov = Eigen::MatrixXd::Zero(a.size(), a.size());
    if (aSigmaA > 1e-15) {
        gradCov = -erfinv_result * (a * a.transpose()) / std::sqrt(aSigmaA);
    }
    result.dSigma  = gradCov;
    result.dLambda = gradCov;

    return result;
}

//--------------------------------------------------
// Barrier check for multiple half-space constraints
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
    for (size_t l = 0; l < a_list.size(); ++l) {
        RiskAwareResult res = riskAwareHalfspaceWithGradient(b, a_list[l], gamma_list[l], delta);

        // Evaluate the risk-aware barrier condition
        double lhs = res.dmu.dot(b_dot.dmu)
                   + (res.dSigma.array()  * b_dot.dSigma.array()).sum()
                   + (res.dLambda.array() * b_dot.dLambda.array()).sum()
                   + res.dmu.dot(B * u);

        // If any one constraint holds, return true
        if (lhs >= -res.h) {
            return true;
        }
    }
    // None satisfied → violated
    return false;
}

bool BarrierTrajectoryValidityChecker::checkTrajectoryBarrierConstraints(
    const State *initial_state,
    const std::vector<ompl::control::Control*> &controls,
    const std::vector<double> &durations) const
{
    bool has_single_obstacle   = !a_list_.empty();
    bool has_multiple_obstacles= !obstacle_a_lists_.empty();

    // Effective delta split across number of obstacles (single=1, multiple=#obstacles)
    double effective_delta = delta_;
    size_t num_obs = 0;
    if (has_single_obstacle) {
        num_obs = 1;
    } else if (has_multiple_obstacles) {
        num_obs = obstacle_a_lists_.size();
    }
    if (num_obs > 0) {
        effective_delta = delta_ / static_cast<double>(num_obs);
    }

    // If no constraints and no controls, nothing to check
    if ((!has_single_obstacle && !has_multiple_obstacles && obstacle_aabbs_.empty()) || controls.empty()) {
        return true;
    }

    // Extract initial belief state
    Belief b;
    extractBeliefState(initial_state, b.mu, b.Sigma, b.Lambda);

    // Check constraints at each control step
    for (size_t i = 0; i < controls.size(); ++i) {
        // Extract control (we use first 2 dims; if control dim is 3, the 3rd is duration handled by OMPL)
        Eigen::VectorXd u;
        extractControl(controls[i], u);

        // Number of time steps for this control duration
        int num_steps = static_cast<int>(durations[i] / dt_);
        if (num_steps < 0) num_steps = 0;

        // Propagate belief and check constraints at each time step
        for (int step = 0; step <= num_steps; ++step) {
            BeliefDerivative b_dot = propagateBelief(b, u, A_, B_, K_, G_, Q_);

            // === Planner barrier constraints (half-space) ===
            bool constraint_satisfied = true;
            if (has_single_obstacle) {
                // Single obstacle: OR over its constraints
                constraint_satisfied = barrierCheckMultiple(b, a_list_, gamma_list_, effective_delta, b_dot, B_, u);
            } else if (has_multiple_obstacles) {
                // Multiple obstacles: AND over obstacles (each obstacle has OR over its faces)
                for (size_t obs_idx = 0; obs_idx < obstacle_a_lists_.size(); ++obs_idx) {
                    bool obs_ok = barrierCheckMultiple(b,
                                                       obstacle_a_lists_[obs_idx],
                                                       obstacle_gamma_lists_[obs_idx],
                                                       effective_delta, b_dot, B_, u);
                    if (!obs_ok) { constraint_satisfied = false; break; }
                }
            }

            if (!constraint_satisfied) {
                return false; // violated planner barrier
            }

            // === Monte-Carlo segment-based collision check against AABBs ===
            // We treat this as *measurement/statistics* by default. If you want it to hard-fail,
            // add a threshold and return false when exceeded.
            if (numSamples_ > 0 && !obstacle_aabbs_.empty()) {
                // 2D mean
                if (b.mu.size() >= 2 && b_dot.dmu.size() >= 2) {
                    Eigen::Vector2d mu1 = b.mu.head<2>();
                    Eigen::Vector2d det_step = dt_ * b_dot.dmu.head<2>();
                    Eigen::Vector2d mu2_det = mu1 + det_step;

                    // Process noise over dt: N(0, (G Q G^T) dt)
                    Eigen::Matrix2d W = (G_ * Q_ * G_.transpose()).topLeftCorner<2,2>();
                    Eigen::Matrix2d cov = dt_ * W;
                    cov += 1e-12 * Eigen::Matrix2d::Identity(); // jitter for PD

                    Eigen::LLT<Eigen::Matrix2d> llt(cov);
                    if (llt.info() == Eigen::Success) {
                        Eigen::Matrix2d L = llt.matrixL();

                        int stepViolations = 0;
                        for (int s = 0; s < numSamples_; ++s) {
                            Eigen::Vector2d z(normal_(rng_), normal_(rng_));
                            Eigen::Vector2d mu2 = mu2_det + L * z;

                            if (segmentHitsAnyAABB(mu1, mu2)) {
                                ++stepViolations;
                            }
                        }
                        totalViolations_ += static_cast<uint64_t>(stepViolations);
                        totalSamples_    += static_cast<uint64_t>(numSamples_);
                        // Debug:
                        // std::cout << "[MC] step " << step << " violations: " << stepViolations << "/" << numSamples_ << std::endl;
                    } else {
                        // Fallback: if covariance not PD, skip MC this step
                    }
                }
            }
            // === end Monte-Carlo ===

            // Euler integration to propagate belief (only if not the last step)
            if (step < num_steps) {
                b.mu     += dt_ * b_dot.dmu;
                b.Sigma  += dt_ * b_dot.dSigma;
                b.Lambda += dt_ * b_dot.dLambda;
            }
        }
    }

    return true; // satisfied for all steps
}

// ---------- Rectangle / AABB helpers ----------

bool BarrierTrajectoryValidityChecker::segmentIntersectsRect(
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1,
    const AABB& r) const
{
    // Quick acceptance: either endpoint inside
    if (inAABB(p0, r) || inAABB(p1, r)) return true;

    // Liang–Barsky line clipping
    Eigen::Vector2d d = p1 - p0;
    double p[4] = {-d.x(), d.x(), -d.y(), d.y()};
    double q[4] = {p0.x() - r.fx, r.tx - p0.x(), p0.y() - r.fy, r.ty - p0.y()};
    double u1 = 0.0, u2 = 1.0;

    for (int i = 0; i < 4; ++i) {
        if (std::abs(p[i]) < 1e-15) {
            if (q[i] < 0.0) return false; // parallel outside
        } else {
            double t = q[i] / p[i];
            if (p[i] < 0.0) {
                if (t > u2) return false;
                if (t > u1) u1 = t;
            } else {
                if (t < u1) return false;
                if (t < u2) u2 = t;
            }
        }
    }
    return u1 <= u2;
}

bool BarrierTrajectoryValidityChecker::inObstacle(const Eigen::VectorXd &x) const
{
    // Prefer AABB check for Monte-Carlo (simple, direct, axis-aligned rectangles)
    if (!obstacle_aabbs_.empty()) {
        if (x.size() < 2) return false;
        Eigen::Vector2d p(x[0], x[1]);
        for (const auto& box : obstacle_aabbs_) {
            if (inAABB(p, box)) return true;
        }
        return false;
    }

    // ---- Fallback: existing half-space logic ----
    if (!a_list_.empty()) {
        // Single obstacle mode (OR over faces)
        for (size_t i = 0; i < a_list_.size(); ++i) {
            if (a_list_[i].dot(x) <= gamma_list_[i]) {
                return true;  // inside obstacle
            }
        }
    } else if (!obstacle_a_lists_.empty()) {
        // Multiple obstacles: inside any one (AND over that obstacle's faces)
        for (size_t obs = 0; obs < obstacle_a_lists_.size(); ++obs) {
            bool inside = true;
            for (size_t j = 0; j < obstacle_a_lists_[obs].size(); ++j) {
                if (obstacle_a_lists_[obs][j].dot(x) > obstacle_gamma_lists_[obs][j]) {
                    inside = false;
                    break;
                }
            }
            if (inside) return true;
        }
    }

    return false; // not in any obstacle
}

// ---------- Extraction helpers ----------

void BarrierTrajectoryValidityChecker::extractBeliefState(const State *state,
                                                          Eigen::VectorXd &mu,
                                                          Eigen::MatrixXd &Sigma,
                                                          Eigen::MatrixXd &Lambda) const
{
    // Try to extract from R2BeliefSpace first
    if (auto belief_state = dynamic_cast<const R2BeliefSpace::StateType*>(state)) {
        mu.resize(2);
        mu << belief_state->getX(), belief_state->getY();
        Sigma  = belief_state->getCovariance();
        Lambda = Eigen::MatrixXd::Zero(2, 2); // Default value
    }
    // Try to extract from RNBeliefSpace
    else if (auto belief_state = dynamic_cast<const RNBeliefSpace::StateType*>(state)) {
        int dim = belief_state->getDimension();
        mu.resize(dim);
        for (int i = 0; i < dim; ++i) mu(i) = belief_state->getComponent(i);
        Sigma  = belief_state->getCovariance();
        Lambda = Eigen::MatrixXd::Zero(dim, dim); // Default value
    }
    // Try to extract from compound state space
    else if (auto compound_state = dynamic_cast<const ompl::base::CompoundStateSpace::StateType*>(state)) {
        if (auto belief_state = dynamic_cast<const R2BeliefSpace::StateType*>(compound_state->components[0])) {
            mu.resize(2);
            mu << belief_state->getX(), belief_state->getY();
            Sigma  = belief_state->getCovariance();
            Lambda = Eigen::MatrixXd::Zero(2, 2);
        } else if (auto belief_state = dynamic_cast<const RNBeliefSpace::StateType*>(compound_state->components[0])) {
            int dim = belief_state->getDimension();
            mu.resize(dim);
            for (int i = 0; i < dim; ++i) mu(i) = belief_state->getComponent(i);
            Sigma  = belief_state->getCovariance();
            Lambda = Eigen::MatrixXd::Zero(dim, dim);
        } else {
            mu     = Eigen::VectorXd::Zero(2);
            Sigma  = Eigen::MatrixXd::Identity(2, 2);
            Lambda = Eigen::MatrixXd::Zero(2, 2);
        }
    } else {
        // Fallback
        mu     = Eigen::VectorXd::Zero(2);
        Sigma  = Eigen::MatrixXd::Identity(2, 2);
        Lambda = Eigen::MatrixXd::Zero(2, 2);
    }
}

void BarrierTrajectoryValidityChecker::extractControl(const ompl::control::Control *control, Eigen::VectorXd &u) const
{
    if (auto rv = dynamic_cast<const ompl::control::RealVectorControlSpace::ControlType*>(control)) {
        // Use the first two components as (vx, vy). Ignore others (e.g., duration) — OMPL handles timing.
        const int want = 2;
        u.resize(want);
        for (int i = 0; i < want; ++i) u(i) = rv->values[i];
    } else {
        // Fallback
        u = Eigen::VectorXd::Zero(2);
    }
}

} // namespace base
} // namespace ompl

