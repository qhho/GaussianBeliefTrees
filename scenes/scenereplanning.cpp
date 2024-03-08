#include "scenereplanning.hpp"

SceneReplanning::SceneReplanning() {
	n_obstacles_ = 1;
	loadConstraints();
	loadDescription();
}

void SceneReplanning::loadConstraints() {
	Eigen::MatrixXf A(6, 3), B(6, 1);

	A << -0, -500, 0, 400, 0, 0, -0, 500, 0, -400, -0, 0, 0, 0, -2000, -0, 0, 2000; A_list_.push_back(A);
    B << -20000, 40000, 40000, -20000, 0, 20000; B_list_.push_back(B);
}

void SceneReplanning::loadDescription() {
	x_offset_ = 0.0; obs_x_n_ = 1; obs_x_size_ = 40.0; inc_x_ = 10.0;
	y_offset_ = 40.0; obs_y_n_ = 1; obs_y_size_ = 30.0; inc_y_ = 17.0;
	z_offset_ = -1.0; obs_z_n_ = 1; obs_z_size_ = 8.0; inc_z_ = 0.0;
}
