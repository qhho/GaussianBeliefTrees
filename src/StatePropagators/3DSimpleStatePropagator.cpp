/* Author: Qi Heng */

#include "../StatePropagators/3DSimpleStatePropagator.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;



ThreeDSimpleStatePropagator::ThreeDSimpleStatePropagator(const oc::SpaceInformationPtr &si, double process_noise) : oc::StatePropagator(si)
{
    duration_ = si->getPropagationStepSize();

    //=========================================================================
    // Open loop system definition
    //=========================================================================
    A_ol_.resize(3, 3);
    A_ol_ << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0;

    B_ol_.resize(3, 3);
    B_ol_ << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0;

    //=========================================================================
    // Close loop system definition
    //=========================================================================
    A_cl_.resize(3, 3);
    A_cl_ = A_ol_ - B_ol_ * K_;

    B_cl_.resize(3, 3);
    B_cl_ = B_ol_ * K_;

    //=========================================================================
    // Discrete close loop system definition
    //=========================================================================
    A_cl_d_.resize(3, 3);
    A_cl_d_ = Eigen::MatrixXd::Identity(3, 3) - A_cl_ * duration_;
    A_cl_d_ = A_cl_d_.inverse().eval();

    // std::cout << A_cl_d_ << std::endl;

    B_cl_d_.resize(3, 3);
    B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXd::Identity(3, 3)) * B_cl_;

    A_BK_(0,0) = 0.7;
    A_BK_(1,1) = 0.7;
    A_BK_(2,2) = 0.7;
    A_BK_(0,1) = 0.0;
    A_BK_(1,0) = 0.0;
    A_BK_(0,2) = 0.0;
    A_BK_(1,2) = 0.0;
    A_BK_(2,0) = 0.0;
    A_BK_(2,1) = 0.0;

    double processNoise = process_noise*duration_; //process noise is 0.0 for scenario
    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
}

void saturate(double &value, const double &min_value, const double &max_value) {
    if(value < min_value) value = min_value;
    if(value > max_value) value = max_value;
}

double wrap(double angle) {
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}

void ThreeDSimpleStatePropagator::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
{
    // use the motionmodel to apply the controls
    // motionModel_->Evolve(state, control, motionModel_->getZeroNoise(), to);

    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    start_css = state->as<R3BeliefSpace::StateType >();

    x_pose = start_css->getX();
    y_pose = start_css->getY();
    z_pose = start_css->getZ();
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    // control_css = control->as<oc::RealVectorControlSpace::ControlType>();

    // x_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    // y_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    // z_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];

    double u_0 = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    double u_1 = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    double u_2 = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];

    // std::cout << "OL control is " << x_pose_reference << " " << y_pose_reference << std::endl;
    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    //=========================================================================
    // double u_0 = B_ol_(0, 0) * (x_pose_reference - x_pose); //double u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose);
    // double u_1 = B_ol_(1, 1) * (y_pose_reference - y_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);
    // double u_2 = B_ol_(2, 2) * (z_pose_reference - z_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);

    // std::cout << u_0 << " " << u_1 << " " << u_2 << std::endl;
    //=========================================================================
    // Bound controller outputs (dot(v) dot(yaw) dot(heave))
    //=========================================================================
    // saturate(u_0, -10.0, 10.0);
    // saturate(u_1, -10.0, 10.0);
    // saturate(u_2, -3.0, 3.0);
    //=========================================================================
    // Propagate mean
    //=========================================================================
    result->as<R3BeliefSpace::StateType>()->setX(x_pose + duration * u_0);
    result->as<R3BeliefSpace::StateType>()->setY(y_pose + duration * u_1);
    result->as<R3BeliefSpace::StateType>()->setZ(z_pose + duration * u_2);

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================

    Eigen::Matrix3d sigma_from = state->as<R3BeliefSpace::StateType>()->getSigma();
    Eigen::Matrix3d lambda_from = state->as<R3BeliefSpace::StateType>()->getLambda();
    Eigen::Matrix3d sigma_pred = F*sigma_from*F + Q;

    // std::cout << A_cl_d_ << std::endl;

    Mat lambda_pred, K;
    // std::cout << x_pose << " " << y_pose << " " << z_pose << std::endl;
    // std::cout << x_pose_reference << " " << y_pose_reference << " " << z_pose_reference << " " << z_pose + duration * u_2 << std::endl;

    if (z_pose + duration * u_2 > 11.0){ //ICRA23 scenario //-75
        Mat R = 0.5*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        Mat S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = A_cl_*lambda_from*A_cl_;
    }
    else{
        K = Eigen::MatrixXd::Zero(dimensions_, dimensions_);
        lambda_pred = lambda_from;
    }
    Mat sigma_to = (I - (K*H)) * sigma_pred;
    Mat lambda_to = lambda_pred + K*H*sigma_pred;

    result->as<R3BeliefSpace::StateType>()->setSigma(sigma_to);
    result->as<R3BeliefSpace::StateType>()->setLambda(lambda_to);
}

bool ThreeDSimpleStatePropagator::canPropagateBackward(void) const
{
    return false;
}