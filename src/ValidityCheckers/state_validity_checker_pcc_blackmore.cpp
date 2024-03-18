#include "ValidityCheckers/state_validity_checker_pcc_blackmore.hpp"

StateValidityCheckerPCCBlackmore::StateValidityCheckerPCCBlackmore(const std::string &scene_id, const ob::SpaceInformationPtr &si, const double p_safe, int sysType) :
	ob::StateValidityChecker(si) {
	si_ = si;
	p_collision_ = 1 - p_safe;

	OMPL_INFORM("Using P collision of %f", p_collision_);

	Scene scene_ = Scene(scene_id);

	sysType_ = sysType;

	n_obstacles_ = scene_.n_obstacles_;
	A_list_.resize(n_obstacles_); A_list_ = scene_.A_list_;
	B_list_.resize(n_obstacles_); B_list_ = scene_.B_list_;

	OMPL_INFORM("scene is %s", scene_id.c_str());

	// OMPL_INFORM("scene is %s", scene_id.c_str());
	// if (scene_id == "2d_narrow") {
	// 	Narrow2D scene = Narrow2D();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
	// }
    // else if (scene_id == "scene4") {
	// 	Scene4 scene = Scene4();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
	// }
	// else if (scene_id == "scenereplanning") {
	// 	SceneReplanning scene = SceneReplanning();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
	// 	std::cout << "scene is scene replanning" << std::endl;
	// }
	// else if (scene_id == "scenereplanning2") {
	// 	SceneReplanning2 scene = SceneReplanning2();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
	// }
	// else if (scene_id == "2d_empty"){
	// 	n_obstacles_ = 0;
	// 	A_list_.resize(n_obstacles_);
	// 	B_list_.resize(n_obstacles_);
	// }
	// else
	// {
	// 	OMPL_INFORM("Unknown scene id");
	// }

	// scene

	if (n_obstacles_ == 0) {
		erf_inv_result_ = computeInverseErrorFunction(1 - 2 * p_collision_);
	}
	else
	erf_inv_result_ = computeInverseErrorFunction(1 - 2 * p_collision_ / n_obstacles_);
}

StateValidityCheckerPCCBlackmore::~StateValidityCheckerPCCBlackmore() {
}

bool StateValidityCheckerPCCBlackmore::isValid(const ob::State *state) const {
	//=========================================================================
	// Bounds checker
	//=========================================================================
	// if (!si_->satisfiesBounds(state)) {
	// 	return false;
	// }

	double x_pose, y_pose, z_pose;
	Eigen::MatrixXf PX(3, 3); PX.setZero();

	if (sysType_ == 0)
	{
		const double x = state->as<R2BeliefSpace::StateType>()->getX();
		const double y = state->as<R2BeliefSpace::StateType>()->getY();
		if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0){
			return false;
		}
		x_pose = state->as<R2BeliefSpace::StateType>()->getX();
		y_pose = state->as<R2BeliefSpace::StateType>()->getY();
		z_pose = 4.0;
		PX(0,0) = state->as<R2BeliefSpace::StateType>()->getCovariance()(0,0);
		PX(1,1) = state->as<R2BeliefSpace::StateType>()->getCovariance()(1,1);
		PX(2,2) = 0.000001;
	}
	else if (sysType_ == 1)
	{
		x_pose = state->as<R3BeliefSpace::StateType>()->getX();
		y_pose = state->as<R3BeliefSpace::StateType>()->getY();
		z_pose = state->as<R3BeliefSpace::StateType>()->getZ();
		if (x_pose > 100.0 || x_pose < 0.0 || y_pose < 0.0 || y_pose > 100.0 || z_pose < 0.0 || z_pose > 100.0){
			return false;
		}
		PX(0,0) = state->as<R3BeliefSpace::StateType>()->getCovariance()(0,0);
		PX(1,1) = state->as<R3BeliefSpace::StateType>()->getCovariance()(1,1);
		PX(2,2) = state->as<R3BeliefSpace::StateType>()->getCovariance()(1,1);
	}
	else if (sysType_ == 2)
	{
		x_pose = state->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX();
		y_pose = state->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY();
		if (x_pose > 100.0 || x_pose < 0.0 || y_pose < 0.0 || y_pose > 100.0){
			return false;
		}
		PX(0,0) = state->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(0,0);
		PX(1,1) = state->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance()(1,1);
		PX(2,2) = 0.00000001;
	}
	else if (sysType_ == 3)
	{
		x_pose = state->as<ob::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getX();
		y_pose = state->as<ob::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getY();
		z_pose = state->as<ob::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getZ();
		if (x_pose > 100.0 || x_pose < 0.0 || y_pose < 0.0 || y_pose > 100.0 || z_pose < 0.0 || z_pose > 100.0){
			return false;
		}
		PX(0,0) = state->as<ob::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getCovariance()(0,0);
		PX(1,1) = state->as<ob::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getCovariance()(1,1);
		PX(2,2) = state->as<ob::CompoundStateSpace::StateType>()->as<R3BeliefSpace::StateType>(0)->getCovariance()(2,2);
	}
	else
		OMPL_ERROR("Unknown system type");
	bool valid = false;

	if (n_obstacles_ == 0) {
		valid = true;
		goto exit_switch;
	}

	for (int o = 0; o < n_obstacles_; o++) {
		if (not HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x_pose, y_pose, z_pose, PX)) {
			goto exit_switch;
		}
	}

	valid = true;

	exit_switch:;
	return valid;
}

bool StateValidityCheckerPCCBlackmore::HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    
    bool valid = false;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * erf_inv_result_;

		if(x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2) >= (B(i, 0) + b_bar)) {
			valid = true;
			break;
		}
	}
	return valid;
}
