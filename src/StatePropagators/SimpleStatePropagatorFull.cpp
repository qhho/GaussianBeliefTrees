/* Author: Qi Heng */

#include "StatePropagators/SimpleStatePropagatorFull.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;



SimpleStatePropagatorFull::SimpleStatePropagatorFull(const oc::SpaceInformationPtr &si, double processNoise, double R, double R_bad, double K_default, std::vector<std::vector<double > > measurement_regions) : oc::StatePropagator(si)
{
    duration_ = si->getPropagationStepSize();
    K_ = K_default;

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

    // double processNoise = 0.2;
    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    R_ = R*R;
    R_bad_ = R_bad*R_bad;

    measurementRegions_ = measurement_regions;
}

void SimpleStatePropagatorFull::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
{
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    start_css = state->as<R2BeliefSpace::StateType >();

    x_pose = start_css->getX();
    y_pose = start_css->getY();
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    // control_css = control->as<oc::RealVectorControlSpace::ControlType>();

    double u_0 = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    double u_1 = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    // K_sample = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];

    Eigen::Matrix2d K_sample;
    K_sample(0, 0) = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];;
    K_sample(0,1) = control->as<oc::RealVectorControlSpace::ControlType>()->values[3];;
    K_sample(1, 0) = control->as<oc::RealVectorControlSpace::ControlType>()->values[4];;
    K_sample(1, 1) = control->as<oc::RealVectorControlSpace::ControlType>()->values[5];;


    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    //=========================================================================
    // double u_0 = B_ol_(0, 0) * (x_pose_reference - x_pose); //double u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose);
    // double u_1 = B_ol_(0, 0) * (y_pose_reference - y_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);

    // //=========================================================================
    // // Get (dot(v) dot(yaw) dot(heave)) from (dot(dot(x)) dot(dot(y)) dot(dot(z)))
    // //=========================================================================
    // u_bar_0 = cos_y * u_0 + sin_y * u_1;
    // u_bar_1 = (-sin_y * u_0 + cos_y * u_1) / surge;
    // u_bar_2 = u_2;

    //=========================================================================
    // Bound controller outputs (dot(v) dot(yaw) dot(heave))
    //=========================================================================
    // saturate(u_0, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
    // saturate(u_1, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
    //=========================================================================
    // Propagate mean
    //=========================================================================
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

    if (x_new > measurementRegions_[0][0] && x_new < measurementRegions_[0][1] && y_new < measurementRegions_[1][1] && y_new < measurementRegions_[1][1]){
        Mat R = R_*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        Mat S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = (A_ol_ - B_ol_ * K_sample)*lambda_from*(A_ol_ - B_ol_ * K_sample);
    }
    else{
        Mat R = R_bad_*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        Mat S = (H * sigma_pred * H.transpose()) + R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred =  (A_ol_ - B_ol_ * K_sample)*lambda_from*(A_ol_ - B_ol_ * K_sample);
        // K = Eigen::MatrixXd::Zero(dimensions_, dimensions_);
        // lambda_pred = lambda_from;
    }
    Mat sigma_to = (I - (K*H)) * sigma_pred;
    Mat lambda_to = lambda_pred + K*H*sigma_pred;

    result->as<R2BeliefSpace::StateType>()->setSigma(sigma_to);
    result->as<R2BeliefSpace::StateType>()->setLambda(lambda_to);
}

bool SimpleStatePropagatorFull::canPropagateBackward(void) const
{
    return false;
}