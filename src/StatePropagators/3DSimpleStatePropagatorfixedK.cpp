/* Author: Qi Heng */

#include "StatePropagators/3DSimpleStatePropagatorfixedK.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;



ThreeDSimpleStatePropagatorFixedK::ThreeDSimpleStatePropagatorFixedK(const oc::SpaceInformationPtr &si,  double processNoise, double R, double R_bad, double K_default, std::vector<std::vector<double > > measurement_regions) : oc::StatePropagator(si)
{
    duration_ = si->getPropagationStepSize();
    K_ = K_default;
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

    // A_BK_(0,0) = K_;
    // A_BK_(1,1) = K_;
    // A_BK_(2,2) = K_;
    // A_BK_(0,1) = 0.0;
    // A_BK_(1,0) = 0.0;
    // A_BK_(0,2) = 0.0;
    // A_BK_(1,2) = 0.0;
    // A_BK_(2,0) = 0.0;
    // A_BK_(2,1) = 0.0;

    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    R_ = R*R;
    R_bad_ = R_bad*R_bad;
}

void ThreeDSimpleStatePropagatorFixedK::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
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
    double u_0 = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    double u_1 = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    double u_2 = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];

    //=========================================================================
    // Propagate mean
    //=========================================================================
    double x_new = x_pose + duration * u_0;
    double y_new = y_pose + duration * u_1;
    double z_new = y_pose + duration + u_2;
    result->as<R3BeliefSpace::StateType>()->setX(x_new);
    result->as<R3BeliefSpace::StateType>()->setY(y_new);
    result->as<R3BeliefSpace::StateType>()->setZ(z_new);

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================

    Eigen::Matrix3d sigma_from = state->as<R3BeliefSpace::StateType>()->getSigma();
    Eigen::Matrix3d lambda_from = state->as<R3BeliefSpace::StateType>()->getLambda();
    Eigen::Matrix3d sigma_pred = F*sigma_from*F + Q;

    // std::cout << A_cl_d_ << std::endl;

    Mat3 lambda_pred, K;
    // std::cout << x_pose << " " << y_pose << " " << z_pose << std::endl;
    // std::cout << x_pose_reference << " " << y_pose_reference << " " << z_pose_reference << " " << z_pose + duration * u_2 << std::endl;

    // if (z_pose + duration * u_2 > 11.0){ //ICRA23 scenario
    if (x_new > measurementRegions_[0][0] && x_new < measurementRegions_[0][1] && y_new > measurementRegions_[1][0] && y_new < measurementRegions_[1][1] && z_new > measurementRegions_[2][0] && z_new < measurementRegions_[2][1]){ 
        Mat3 R = R_*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        Mat3 S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = A_cl_*lambda_from*A_cl_; //lambda_pred = A_cl_*lambda_from*A_cl_;
    }
    else{
        Mat3 R = R_bad_*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        Mat3 S = (H * sigma_pred * H.transpose()) + R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred =  A_cl_*lambda_from*A_cl_;
        // K = Eigen::MatrixXd::Zero(dimensions_, dimensions_);
        // lambda_pred = lambda_from;
    }
    Mat3 sigma_to = (I - (K*H)) * sigma_pred;
    Mat3 lambda_to = lambda_pred + K*H*sigma_pred;

    result->as<R3BeliefSpace::StateType>()->setSigma(sigma_to);
    result->as<R3BeliefSpace::StateType>()->setLambda(lambda_to);
}

bool ThreeDSimpleStatePropagatorFixedK::canPropagateBackward(void) const
{
    return false;
}