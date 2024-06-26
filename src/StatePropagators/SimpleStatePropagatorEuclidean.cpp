/* Author: Qi Heng */

#include "StatePropagators/SimpleStatePropagatorEuclidean.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;



SimpleStatePropagatorEuclidean::SimpleStatePropagatorEuclidean(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
{
    duration_ = si->getPropagationStepSize();

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

    double processNoise = 0.0; //secnario 3 0.6; //secnario 1
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

void SimpleStatePropagatorEuclidean::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
{
    // use the motionmodel to apply the controls
    // motionModel_->Evolve(state, control, motionModel_->getZeroNoise(), to);

    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    start_css = state->as<R2BeliefSpaceEuclidean::StateType >();

    x_pose = start_css->getX();
    y_pose = start_css->getY();
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    // control_css = control->as<oc::RealVectorControlSpace::ControlType>();

    x_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    y_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];

    // std::cout << "OL control is " << x_pose_reference << " " << y_pose_reference << std::endl;
    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    //=========================================================================
    double u_0 = B_ol_(0, 0) * (x_pose_reference - x_pose); //double u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose);
    double u_1 = B_ol_(0, 0) * (y_pose_reference - y_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);


    // u_0 = u_0/(u_0 + u_1);
    // std::cout << "PD control is " << u_0 << " " << u_1 << std::endl;
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
    // result_css = result->as<R2BeliefSpace::StateType>();
    // std::cout << "x_pose: " << duration * x_pose_reference << std::endl;
    // std::cout << "y_pose: " << duration * y_pose_reference << std::endl;

    // std::cout << duration << std::endl;
    // if (duration > 0.1){
    //     std::cout << duration << std::endl;
    // }
    result->as<R2BeliefSpaceEuclidean::StateType>()->setX(x_pose + duration * x_pose_reference);
    result->as<R2BeliefSpaceEuclidean::StateType>()->setY(y_pose + duration * y_pose_reference);

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================
    // std::cout << result_css << std::endl;
    // result_css->as<R2BeliefSpace::StateType>()->getSigma();


    Eigen::Matrix2d sigma_from = state->as<R2BeliefSpaceEuclidean::StateType>()->getSigma();
    Eigen::Matrix2d lambda_from = state->as<R2BeliefSpaceEuclidean::StateType>()->getLambda();
    Eigen::Matrix2d sigma_pred = F*sigma_from*F + Q;

    Mat lambda_pred, K;

    // if (x_pose > 75 && y_pose  < 30){
    //     std::cout << "old state is in measurement region!" << std::endl;
    // }

    //scenario 1
    // if (x_pose < -100.0){
    // if (x_pose + duration * u_0 > 75 && y_pose + duration * u_1 < 30){ //scenario 2
    // if (true){ //scenario 3
    if (x_pose + duration * u_0 > 0.0 && x_pose + duration * u_0 < 55 && y_pose + duration * u_1 < 25){  //scenario 4
        Mat R = 0.001*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        // Mat R = 1*((x_pose - 87.5)*(x_pose - 87.5) + (y_pose - 15.0)*(y_pose - 15.0) +1)*Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        Mat S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = A_cl_*lambda_from*A_cl_;

        // std::cout << "MEASUREMENT REGION" <<std::endl;
        // std::cout << "S is:  " << S << std::endl;
        // std::cout << "K is: " << K << std::endl;
    }
    else{
        K = Eigen::MatrixXd::Zero(dimensions_, dimensions_);
        lambda_pred = lambda_from;
    }
    Mat sigma_to = (I - (K*H)) * sigma_pred;
    Mat lambda_to = lambda_pred + K*H*sigma_pred;

    
    // if (y_pose + duration * y_pose_reference > 80) {
    //     std::cout << "x is: " << x_pose + duration * x_pose_reference << std::endl;
    //     std::cout << "y is: " << y_pose + duration * y_pose_reference << std::endl;
    // }
    result->as<R2BeliefSpaceEuclidean::StateType>()->setSigma(sigma_to);
    result->as<R2BeliefSpaceEuclidean::StateType>()->setLambda(lambda_to);

    // if (x_pose + duration * u_0 > 75 && y_pose + duration * u_1 < 30){
    //     if (x_pose + duration * u_0 > 0.0 && x_pose + duration * u_0 < 55 && y_pose + duration * u_1 < 25){ 
    //     result->as<R2BeliefSpaceEuclidean::StateType>()->setSigma(0.01*Eigen::MatrixXd::Identity(2,2));
    //     result->as<R2BeliefSpaceEuclidean::StateType>()->setLambda(0.01*Eigen::MatrixXd::Identity(2,2));
    // }

    // std::cout << result->as<R2BeliefSpaceEuclidean::StateType>()->getCovariance().trace() << std::endl;
    // std::cout << "sigma_from is: " << sigma_from << std::endl;
    // std::cout << "sigma_to is: " << sigma_to << std::endl;
    // std::cout << "lambda from is: " << lambda_from <<std::endl;
    // std::cout << "K is: " << K << std::endl;
    // std::cout << "H is: " << H << std::endl;
    // double x = result->as<R2BeliefSpace::StateType>()->getX();
    // double y = result->as<R2BeliefSpace::StateType>()->getY();
    
    // std::cout << (sigma_to + lambda_to).trace() << std::endl;
    // 
}

bool SimpleStatePropagatorEuclidean::canPropagateBackward(void) const
{
    return false;
}