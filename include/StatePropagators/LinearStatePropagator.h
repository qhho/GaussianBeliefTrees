#ifndef LINEAR_RN_BELIEF_PROPAGATOR_H
#define LINEAR_RN_BELIEF_PROPAGATOR_H

#include "ompl/control/SpaceInformation.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "Spaces/RNBeliefSpace.h"
#include <Eigen/Dense>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;

typedef Eigen::MatrixXd MatXd;

/** \brief State propagation for a N-dimensional linear system with belief space. */
class LinearRNBeliefPropagator : public oc::StatePropagator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** \brief Construct a linear system state propagator with identity matrices
     * @param si Space information
     * @param stateDim Dimension of the state space
     * @param processNoise Process noise standard deviation
     * @param R Measurement noise standard deviation in good regions
     * @param R_bad Measurement noise standard deviation in bad regions
     * @param K_default Default feedback gain
     * @param measurement_regions Regions with good measurements [[x_min, x_max], [y_min, y_max], ...]
     */
    LinearRNBeliefPropagator(const oc::SpaceInformationPtr &si, 
                          unsigned int stateDim,
                          double processNoise, 
                          double R, 
                          double R_bad, 
                          double K_default, 
                          std::vector<std::vector<double>> measurement_regions);
    
    /** \brief Construct a linear system state propagator with custom system matrices
     * @param si Space information
     * @param A_ol Open-loop system matrix
     * @param B_ol Open-loop control matrix
     * @param processNoise Process noise standard deviation
     * @param R Measurement noise standard deviation in good regions
     * @param R_bad Measurement noise standard deviation in bad regions
     * @param K_default Default feedback gain
     * @param measurement_regions Regions with good measurements [[x_min, x_max], [y_min, y_max], ...]
     */
    LinearRNBeliefPropagator(const oc::SpaceInformationPtr &si, 
                          const Eigen::MatrixXd &A_ol,
                          const Eigen::MatrixXd &B_ol,
                          double processNoise, 
                          double R, 
                          double R_bad, 
                          double K_default, 
                          std::vector<std::vector<double>> measurement_regions);

    virtual ~LinearRNBeliefPropagator(void) {}

    /** \brief Will always return false, as the system cannot propagate backward in time due to uncertainty */
    virtual bool canPropagateBackward(void) const;

    /** \brief Propagate from a state, under a given control, for some specified amount of time.
     * @param state The initial state
     * @param control The control to apply
     * @param duration The duration to apply the control for
     * @param result The resulting state after propagation
     */
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const;

private:
    // System matrices
    Eigen::MatrixXd A_ol_, B_ol_, A_cl_, B_cl_, A_cl_d_, B_cl_d_;
    
    // Propagation step size
    double duration_;

    // For state extraction (mutable to be used in const methods)
    mutable Eigen::VectorXd current_state;
    mutable double K_sample;

    // Identity and measurement matrices
    Eigen::MatrixXd I;  // Identity matrix
    Eigen::MatrixXd H;  // Measurement matrix
    Eigen::MatrixXd F;  // State transition matrix (typically identity)

    // Noise parameters
    Eigen::MatrixXd Q;  // Process noise covariance
    double R_;          // Measurement noise variance (good region)
    double R_bad_;      // Measurement noise variance (bad region)

    // Default feedback gain
    double K_default_;

    // Measurement regions
    std::vector<std::vector<double>> measurementRegions_;

protected:
    // Dimension of the state space
    unsigned int dimensions_;
    unsigned int controlDim_;
};

#endif // LINEAR_RN_BELIEF_PROPAGATOR_H