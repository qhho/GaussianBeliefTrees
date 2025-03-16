/* Author: Qi Heng Ho, Èric Pairet
 * Date:   28 February 2018
 *
 * Description:
 */
#include "StatePropagators/3DUnicyclePropagatorfixedK.h"
#include <boost/math/special_functions/erf.hpp>

DynUnicycleControlSpace3DFixedK::DynUnicycleControlSpace3DFixedK(const oc::SpaceInformationPtr &si, double processNoise, double R, double R_bad, double K_default, std::vector<std::vector<double > > measurement_regions) : oc::StatePropagator(si) {
    
    myK_ = K_default;
    forward_acceleration_bounds_.push_back(-1.0);
    forward_acceleration_bounds_.push_back(1.0);
    turning_rate_bounds_.push_back(-1.5);
    turning_rate_bounds_.push_back(1.5);
    heave_acceleration_bounds_.push_back(-3.0);
    heave_acceleration_bounds_.push_back(3.0);
    surge_bounds_.push_back(0.05);
    surge_bounds_.push_back(2.0);
    heave_bounds_.push_back(-2.0);
    heave_bounds_.push_back(2.0);

    controller_parameters_.push_back(0.1167);
    controller_parameters_.push_back(0.131);
    controller_parameters_.push_back(0.1167);
    controller_parameters_.push_back(0.131);
    controller_parameters_.push_back(0.1167);
    controller_parameters_.push_back(0.131);
    
    duration_ = si->getPropagationStepSize();
    // std::cout << "trying this" << std::endl;
    //=========================================================================
    // Open loop system definition
    //=========================================================================
    A_ol_.resize(6, 6);
    A_ol_ << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    B_ol_.resize(6, 3);
    B_ol_ << 0.0, 0.0, 0.0,
             1.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 1.0;
    //=========================================================================
    // PD controller coefficients (from LQR)
    //=========================================================================
    K_.resize(3, 6); K_.setZero();
    K_(0, 0) = controller_parameters_[0]; // Gain P in x
    K_(0, 1) = controller_parameters_[1]; // Gain D in x
    K_(1, 2) = controller_parameters_[2]; // Gain P in y
    K_(1, 3) = controller_parameters_[3]; // Gain D in y
    K_(2, 4) = controller_parameters_[4]; // Gain P in z
    K_(2, 5) = controller_parameters_[5]; // Gain D in z
    // std::cout << "trying this" << std::endl;
    //=========================================================================
    // Close loop system definition
    //=========================================================================
    A_cl_.resize(6, 6);
    A_cl_ = A_ol_ - B_ol_ * K_;

    B_cl_.resize(6, 6);
    B_cl_ = B_ol_ * K_;

    //=========================================================================
    // Discrete close loop system definition
    //=========================================================================
    A_cl_d_.resize(6, 6);
    A_cl_d_ = Eigen::MatrixXf::Identity(6, 6) + A_cl_ * duration_;

    B_cl_d_.resize(6, 6);
    B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXf::Identity(6, 6)) * B_cl_;

    Eigen::MatrixXf A(6, 3), B(6, 1);

    A << -0, -1000, 0, 1000, 0, 0, -0, 1000, 0, -1000, -0, 0, 0, 0, -10000, -0, 0, 10000; A_list_.push_back(A);
    B << 0, 100000, 100000, 0, -100000, 200000; B_list_.push_back(B);

    // set a 2 by 2 matrix for sigma uncertainty propagation
    A_cl_d_33_.resize(3, 3);
    A_cl_d_33_(0, 0) = A_cl_d_(0, 0);
    A_cl_d_33_(0, 1) = A_cl_d_(0, 2);
    A_cl_d_33_(1, 0) = A_cl_d_(2, 0);
    A_cl_d_33_(1, 1) = A_cl_d_(2, 2);
    A_cl_d_33_(0, 2) = A_cl_d_(0, 4);
    A_cl_d_33_(2, 0) = A_cl_d_(4, 0);
    A_cl_d_33_(1, 2) = A_cl_d_(2, 4);
    A_cl_d_33_(2, 1) = A_cl_d_(4, 2);
    A_cl_d_33_(2, 2) = A_cl_d_(4, 4);

    A_BK_(0,0) = myK_;
    A_BK_(1,1) = myK_;
    A_BK_(2,2) = myK_;
    A_BK_(0,1) = 0.0;
    A_BK_(1,0) = 0.0;
    A_BK_(0,2) = 0.0;
    A_BK_(1,2) = 0.0;
    A_BK_(2,0) = 0.0;
    A_BK_(2,1) = 0.0;

    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    R_ = R*R;
    R_bad_ = R_bad*R_bad;
}

void saturate3d(double &value, const double &min_value, const double &max_value) {
    if(value < min_value) value = min_value;
    if(value > max_value) value = max_value;
}

double wrap3d(double angle) {
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}


bool TheHyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, double x_pose, double y_pose, double z_pose, const Eigen::MatrixXf &PX) {
	
    bool valid = true;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * boost::math::erf_inv(1 - 2 * 0.6/6);;

		if(B(i, 0) - (x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2)) >= b_bar) {
			// valid = false;
            // return valid;
            valid = true;
		}
        else{
            valid = false;
            return false;
        }
	}
	return valid;
}

void DynUnicycleControlSpace3DFixedK::propagate(const ob::State *start, const oc::Control* control, const double duration, ob::State *result) const {
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================
    start_css = start->as<ob::CompoundStateSpace::StateType>();

    x_pose = start_css->as<R3BeliefSpace::StateType>(0)->values[0];
    y_pose = start_css->as<R3BeliefSpace::StateType>(0)->values[1];
    z_pose = start_css->as<R3BeliefSpace::StateType>(0)->values[2];
    yaw = start_css->as<ob::SO2StateSpace::StateType>(1)->value;
    surge = start_css->as<ob::RealVectorStateSpace::StateType>(2)->values[0];
    heave = start_css->as<ob::RealVectorStateSpace::StateType>(3)->values[0];
    // Pxx_init = start_css_rvs_cov->values[0];
    // Pyy_init = start_css_rvs_cov->values[1];
    // Pzz_init = start_css_rvs_cov->values[2];

    cos_y = cos(yaw);
    sin_y = sin(yaw);

    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================
    control_css = control->as<oc::CompoundControlSpace::ControlType>();
    control_css_rvs_pose = control_css->as<oc::RealVectorControlSpace::ControlType>(0);

    x_pose_reference = control_css_rvs_pose->values[0];
    y_pose_reference = control_css_rvs_pose->values[1];
    z_pose_reference = control_css_rvs_pose->values[2];
    yaw_reference = control_css->as<oc::RealVectorControlSpace::ControlType>(1)->values[0];
    surge_reference = control_css->as<oc::RealVectorControlSpace::ControlType>(2)->values[0];
    heave_reference = control_css->as<oc::RealVectorControlSpace::ControlType>(3)->values[0];

    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    //=========================================================================
    // does it make sense to try to achieve the velocity of the randomly sampled state?
    // u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose) + B_cl_d_(0, 1) * (surge_reference * cos(yaw_reference) - surge * cos_y);
    // u_1 = B_cl_d_(2, 2) * (y_pose_reference - y_pose) + B_cl_d_(2, 3) * (surge_reference * sin(yaw_reference) - surge * sin_y);
    // u_2 = B_cl_d_(4, 4) * (z_pose_reference - z_pose) + B_cl_d_(4, 5) * (heave_reference - heave);

    u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose) + B_cl_d_(0, 1) * (surge_reference * cos(yaw_reference) - surge * cos_y);
    u_1 = B_cl_d_(2, 2) * (y_pose_reference - y_pose) + B_cl_d_(2, 3) * (surge_reference * sin(yaw_reference) - surge * sin_y);
    u_2 = B_cl_d_(4, 4) * (z_pose_reference - z_pose) + B_cl_d_(4, 5) * (heave_reference - heave);

    //=========================================================================
    // Get (dot(v) dot(yaw) dot(heave)) from (dot(dot(x)) dot(dot(y)) dot(dot(z)))
    //=========================================================================
    u_bar_0 = cos_y * u_0 + sin_y * u_1;
    u_bar_1 = (-sin_y * u_0 + cos_y * u_1) / surge;
    u_bar_2 = u_2;

    //=========================================================================
    // Bound controller outputs (dot(v) dot(yaw) dot(heave))
    //=========================================================================
    saturate3d(u_bar_0, forward_acceleration_bounds_[0], forward_acceleration_bounds_[1]);
    saturate3d(u_bar_1, turning_rate_bounds_[0], turning_rate_bounds_[1]);
    saturate3d(u_bar_2, heave_acceleration_bounds_[0], heave_acceleration_bounds_[1]);

    //=========================================================================
    // Bound surge and heave
    //=========================================================================
    surge_final = surge + duration * u_bar_0;
    saturate3d(surge_final, surge_bounds_[0], surge_bounds_[1]);
    heave_final = heave + duration * u_bar_2;
    saturate3d(heave_final, heave_bounds_[0], heave_bounds_[1]);

    //=========================================================================
    // Propagate mean
    //=========================================================================
    result_css = result->as<ob::CompoundStateSpace::StateType>();
    R3BeliefSpace::StateType *result_css_rvs_pose = result_css->as<R3BeliefSpace::StateType>(0);

    result_css_rvs_pose->values[0] = x_pose + duration * cos_y * surge_final;
    result_css_rvs_pose->values[1] = y_pose + duration * sin_y * surge_final;
    result_css_rvs_pose->values[2] = z_pose + duration * heave_final;
    result_css->as<ob::SO2StateSpace::StateType>(1)->value = wrap3d(yaw + duration * u_bar_1);
    result_css->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = surge_final;
    result_css->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = heave_final;


    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================
    ob::RealVectorStateSpace::StateType *result_css_rvs_cov = result_css->as<ob::RealVectorStateSpace::StateType>(3);

    Eigen::Matrix3d sigma_from = start_css->as<R3BeliefSpace::StateType>(0)->getSigma();
    Eigen::Matrix3d lambda_from = start_css->as<R3BeliefSpace::StateType>(0)->getLambda();
    Eigen::Matrix3d sigma_pred = A_cl_d_33_*sigma_from*A_cl_d_33_ + Q;

    Mat3 K, lambda_pred;

    Eigen::Matrix3f PX; PX.setZero();
    PX(0,0) = sigma_pred(0,0);
    PX(1,1) = sigma_pred(1,1);
    PX(2,2) = sigma_pred(2,2);

    if (TheHyperplaneCCValidityChecker(A_list_.at(0), B_list_.at(0), result_css_rvs_pose->getX(), result_css_rvs_pose->getY(), result_css_rvs_pose->getZ(), PX)) {
        Mat3 R = R_*Eigen::Matrix3d::Identity();
        Eigen::Matrix3d S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = A_BK_*lambda_from*A_BK_;
    }
    else{
        Mat3 R = R_bad_*Eigen::Matrix3d::Identity();
        Eigen::Matrix3d S = (H * sigma_pred * H.transpose())+ R;
        K = (sigma_pred * H.transpose()) * S.inverse();
        lambda_pred = A_BK_*lambda_from*A_BK_;
        // K = Eigen::Matrix3d::Zero(dimensions_, dimensions_);
        // lambda_pred = lambda_from;
    }
    Eigen::Matrix3d sigma_to = (I - (K*H)) * sigma_pred;
    Eigen::Matrix3d lambda_to = lambda_pred + K*H*sigma_pred;

    result_css_rvs_pose->setSigma(sigma_to);
    result_css_rvs_pose->setLambda(lambda_to);
}
