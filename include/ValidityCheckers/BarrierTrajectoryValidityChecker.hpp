#pragma once

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <Eigen/Dense>

#include <vector>
#include <random>
#include <cstdint>

namespace ompl {
namespace base {

class BarrierTrajectoryValidityChecker : public StateValidityChecker
{
public:
    using SpaceInformationPtr = ompl::control::SpaceInformationPtr;

    struct AABB {
        double fx; // min x
        double tx; // max x
        double fy; // min y
        double ty; // max y
    };

    explicit BarrierTrajectoryValidityChecker(const SpaceInformationPtr &si);

    // OMPL interface
    bool isValid(const State *state) const override;

    // Trajectory-level checker (used by planner/validation)
    bool isValidTrajectory(const State *initial_state,
                           const std::vector<ompl::control::Control*> &controls,
                           const std::vector<double> &durations) const;

    // System setup
    void setSystemMatrices(const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &B,
                           const Eigen::MatrixXd &K,
                           const Eigen::MatrixXd &G,
                           const Eigen::MatrixXd &Q);

    // Single obstacle (set of half-spaces)
    void setHalfSpaceConstraints(const std::vector<Eigen::VectorXd> &a_list,
                                 const std::vector<double> &gamma_list,
                                 double delta);

    // Multiple obstacles (each obstacle is a set of half-spaces)
    void setMultipleObstacles(const std::vector<std::vector<Eigen::VectorXd>> &obstacle_a_lists,
                              const std::vector<std::vector<double>> &obstacle_gamma_lists,
                              double delta);

    // Time discretization used by the checker
    void setTimeParameters(int N, double dt);

    // -------- Monte Carlo controls & stats --------
    // Number of MC samples per propagation step
    void setNumSamples(int n) { numSamples_ = (n < 0 ? 0 : n); }
    int  getNumSamples() const { return numSamples_; }

    // AABBs as simple rectangular obstacles for MC intersection checks
    void clearAABBs() { obstacle_aabbs_.clear(); }
    void addAABB(double fx, double tx, double fy, double ty) {
        obstacle_aabbs_.push_back(AABB{fx, tx, fy, ty});
    }
    const std::vector<AABB>& getAABBs() const { return obstacle_aabbs_; }

    // Empirical stats across all MC calls during this run
    void   resetEmpiricalStats() const { totalViolations_ = 0; totalSamples_ = 0; }
    double getEmpiricalPsatisfy() const {
        if (totalSamples_ == 0) return 1.0;
        return 1.0 - static_cast<double>(totalViolations_) / static_cast<double>(totalSamples_);
    }
    void   getEmpiricalCounts(std::uint64_t &violations, std::uint64_t &samples) const {
        violations = totalViolations_;
        samples    = totalSamples_;
    }

private:
    // Core routine used by isValidTrajectory
    bool checkTrajectoryBarrierConstraints(const State *initial_state,
                                           const std::vector<ompl::control::Control*> &controls,
                                           const std::vector<double> &durations) const;

    // State/control extraction helpers
    void extractBeliefState(const State *state,
                            Eigen::VectorXd &mu,
                            Eigen::MatrixXd &Sigma,
                            Eigen::MatrixXd &Lambda) const;

    void extractControl(const ompl::control::Control *control, Eigen::VectorXd &u) const;

    // Geometric collision helpers (AABB-based) used by MC
    inline bool inAABB(const Eigen::Vector2d& p, const AABB& r) const {
        return (p.x() >= r.fx && p.x() <= r.tx && p.y() >= r.fy && p.y() <= r.ty);
    }
    bool segmentIntersectsRect(const Eigen::Vector2d& p0,
                               const Eigen::Vector2d& p1,
                               const AABB& r) const;

    inline bool segmentHitsAnyAABB(const Eigen::Vector2d& p0,
                                   const Eigen::Vector2d& p1) const {
        for (const auto& box : obstacle_aabbs_) {
            if (segmentIntersectsRect(p0, p1, box)) return true;
        }
        return false;
    }

    // Half-space fallback used by legacy planner checks and optional point tests
    bool inObstacle(const Eigen::VectorXd &x) const;

private:
    // -------- System matrices for belief propagation --------
    Eigen::MatrixXd A_, B_, K_, G_, Q_;

    // -------- Chance-constraint params --------
    double delta_{0.01}; // per-obstacle chance threshold (split effectively inside)
    int    N_{10};
    double dt_{0.1};

    // -------- Single obstacle (half-spaces) --------
    std::vector<Eigen::VectorXd> a_list_;
    std::vector<double>          gamma_list_;

    // -------- Multiple obstacles (each is a set of half-spaces) --------
    std::vector<std::vector<Eigen::VectorXd>> obstacle_a_lists_;
    std::vector<std::vector<double>>          obstacle_gamma_lists_;

    // -------- AABB rectangles for Monte-Carlo segment checks --------
    std::vector<AABB> obstacle_aabbs_;

    // -------- Monte-Carlo controls & RNG --------
    int numSamples_{1000};
    mutable std::mt19937 rng_{5489u};                 // deterministic by default
    mutable std::normal_distribution<double> normal_{0.0, 1.0};

    // Empirical stats across all calls
    mutable std::uint64_t totalViolations_{0};
    mutable std::uint64_t totalSamples_{0};
};

} // namespace base
} // namespace ompl
