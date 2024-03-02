/* Author: Qi
 * Date:   28 February 2018
 *
 * Description:
 */

#include "3d_state_validity_checker_pcc_blackmore_simple.hpp"

StateValidityCheckerPCCBlackmoreSimple3D::StateValidityCheckerPCCBlackmoreSimple3D(const std::string &scene_id, const ob::SpaceInformationPtr &si, const double p_safe) :
	ob::StateValidityChecker(si) {
	si_ = si;
	p_collision_ = 1 - p_safe;

	OMPL_INFORM("scene is %s", scene_id.c_str());
	if (scene_id == "scene3d") {
		Scene3d scene = Scene3d();
		n_obstacles_ = scene.n_obstacles_;
		A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
		B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
	}
    // else if (scene_id == "scene4") {
	// 	Scene4 scene = Scene4();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
    //     // std::cout << "done" << std::endl;
    //     // std::cout << A_list_(0,0) << std::endl;
	// }
	// else if (scene_id == "scenereplanning") {
	// 	SceneReplanning scene = SceneReplanning();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
	// 	std::cout << "scene is scene replanning" << std::endl;
    //     // std::cout << "done" << std::endl;
    //     // std::cout << A_list_(0,0) << std::endl;
	// }
	// else if (scene_id == "scenereplanning2") {
	// 	SceneReplanning2 scene = SceneReplanning2();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
    //     // std::cout << "done" << std::endl;
    //     // std::cout << A_list_(0,0) << std::endl;
	// }

	erf_inv_result_ = computeInverseErrorFunction(1 - 2 * p_collision_ / n_obstacles_);
    std::cout << "done" << std::endl;
}

StateValidityCheckerPCCBlackmoreSimple3D::~StateValidityCheckerPCCBlackmoreSimple3D() {
}

bool circlesRect(float cx, float cy, float radius, float rx, float ry, float rw, float rh) {

  // temporary variables to set edges for testing
  float testX = cx;
  float testY = cy;

  // which edge is closest?
  if (cx < rx)         testX = rx;      // test left edge
  else if (cx > rx+rw) testX = rx+rw;   // right edge
  if (cy < ry)         testY = ry;      // top edge
  else if (cy > ry+rh) testY = ry+rh;   // bottom edge

  // get distance from closest edges
  float distX = cx-testX;
  float distY = cy-testY;
  float distance = sqrt( (distX*distX) + (distY*distY) );

  // if the distance is less than the radius, collision!
  if (distance <= radius) {
    return true;
  }
  return false;
}

bool StateValidityCheckerPCCBlackmoreSimple3D::isValid(const ob::State *state) const {
    // std::cout << "checking" << std::endl;
	//=========================================================================
	// Bounds checker
	//=========================================================================
	// if (!si_->satisfiesBounds(state)) {
	// 	// std::cout << "state refused! Reason: out of bounds!" << std::endl;
	// 	return false;
	// }

	// return true;
    const double x = state->as<R3BeliefSpace::StateType>()->getX();
    const double y = state->as<R3BeliefSpace::StateType>()->getY();
	const double z = state->as<R3BeliefSpace::StateType>()->getZ();
    if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0 || z < 0.0 || z > 15.0){
        return false;
    }

	// if (x > 40.0 && x < 60.0 && y > 0 && y < 50){
	// 	return false;
	// }

	// return true;
	// if (state->as<R2BeliefSpace::StateType>()->getCovariance().trace() > 10.0){
	// 	return false;
	// }
	// return true;

	//=========================================================================
	// Extract the component of the state and cast it to what we expect
	//=========================================================================
	double x_pose, y_pose, z_pose;
	Eigen::MatrixXf PX(3, 3); PX.setZero();

	x_pose = state->as<R3BeliefSpace::StateType>()->getX();
	y_pose = state->as<R3BeliefSpace::StateType>()->getY();
	z_pose = state->as<R3BeliefSpace::StateType>()->getZ();;
    PX(0,0) = state->as<R3BeliefSpace::StateType>()->getCovariance()(0,0);
    PX(1,1) = state->as<R3BeliefSpace::StateType>()->getCovariance()(1,1);
    PX(2,2) = state->as<R3BeliefSpace::StateType>()->getCovariance()(2,2);
	// PX(0,0) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[0];
	// PX(1,1) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[1];
	// PX(2,2) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[2];
	double covariance_elem = PX(0,0);
	// if(circlesRect(x_pose, y_pose, 2.477*sqrt(covariance_elem), 0.0, 100.0, 100.0, 10.0)){
	// 	// std::cout << x_pose << " " << y_pose << " " << sqrt(covariance_elem)/2.0 << std::endl;
	// 	return false;
	// }
	// else if(circlesRect(x_pose, y_pose, 2.477*sqrt(covariance_elem), -10.0, -40.0, 10.0, 100.0)){
	// 	// std::cout << x_pose << " " << y_pose << " " << sqrt(covariance_elem)/2.0 << std::endl;
	// 	return false;
	// }
	// else if(circlesRect(x_pose, y_pose, 2.477*sqrt(covariance_elem), 100.0, -40.0, 10.0, 100.0)){
	// 	// std::cout << x_pose << " " << y_pose << " " << sqrt(covariance_elem)/2.0 << std::endl;
	// 	return false;
	// }

	//=========================================================================
	// Probabilistic collision checker
	//=========================================================================
	bool valid = false;
	for (int o = 0; o < n_obstacles_; o++) {
		if (not HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x_pose, y_pose, z_pose, PX)) {
			goto exit_switch;
		}
	}
	valid = true;

	exit_switch:;
	return valid;
}

bool StateValidityCheckerPCCBlackmoreSimple3D::HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    
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
