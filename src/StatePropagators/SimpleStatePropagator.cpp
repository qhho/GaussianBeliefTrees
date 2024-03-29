/* Author: Qi Heng */

#include "StatePropagators/SimpleStatePropagator.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;



SimpleStatePropagator::SimpleStatePropagator(const oc::SpaceInformationPtr &si, double processNoise, double R) : oc::StatePropagator(si)
{
    duration_ = si->getPropagationStepSize();
    K_ = 0.8;

    //=========================================================================
    // Open loop system definition
    //=========================================================================
    A_ol_.resize(2, 2);
    A_ol_ << 1.0, 0.0,
             0.0, 1.0;

    B_ol_.resize(2, 2);
    B_ol_ << 1.0, 0.0,
             0.0, 1.0;

    //=========================================================================
    // Close loop system definition
    //=========================================================================
    A_cl_.resize(2, 2);
    A_cl_ = A_ol_ - B_ol_ * K_;

    B_cl_.resize(2, 2);
    B_cl_ = B_ol_ * K_;

    //=========================================================================
    // Discrete close loop system definition
    //=========================================================================
    A_cl_d_.resize(2, 2);
    A_cl_d_ = Eigen::MatrixXd::Identity(2, 2) + A_cl_ * duration_;

    B_cl_d_.resize(2, 2);
    B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXd::Identity(2, 2)) * B_cl_;

    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    R_ = R*R;
}

void SimpleStatePropagator::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
{
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    start_css = state->as<R2BeliefSpace::StateType >();

    x_pose = start_css->getX();
    y_pose = start_css->getY();
    double u_0 = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    double u_1 = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    K_sample = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];
    double x_new = x_pose + duration * u_0;
    double y_new = y_pose + duration * u_1;
    result->as<R2BeliefSpace::StateType>()->setX(x_new);
    result->as<R2BeliefSpace::StateType>()->setY(y_new);

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================

    Eigen::Matrix2d sigma_from = state->as<R2BeliefSpace::StateType>()->getSigma();
    Eigen::Matrix2d lambda_from = state->as<R2BeliefSpace::StateType>()->getLambda();
    Eigen::Matrix2d sigma_pred = F*sigma_from*F + Q;

    Mat lambda_pred, K;

    // if (x_new > measurementRegions_[0][0] && x_new < measurementRegions_[0][1] && y_new < measurementRegions_[1][1] && y_new < measurementRegions_[1][1]){
    Mat R = R_*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    Mat S = (H * sigma_pred * H.transpose())+ R;
    K = (sigma_pred * H.transpose()) * S.inverse();
    lambda_pred = (A_ol_ - B_ol_ * K_sample)*lambda_from*(A_ol_ - B_ol_ * K_sample);
    // }
    // else{
        // K = Eigen::MatrixXd::Zero(dimensions_, dimensions_);
        // lambda_pred = lambda_from;
    // }
    Mat sigma_to = (I - (K*H)) * sigma_pred;
    Mat lambda_to = lambda_pred + K*H*sigma_pred;

    result->as<R2BeliefSpace::StateType>()->setSigma(sigma_to);
    result->as<R2BeliefSpace::StateType>()->setLambda(lambda_to);
}

bool SimpleStatePropagator::canPropagateBackward(void) const
{
    return false;
}