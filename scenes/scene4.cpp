#include "scene4.hpp"

Scene4::Scene4() {
	n_obstacles_ = 2;
	loadConstraints();
	loadDescription();
}

void Scene4::loadConstraints() {
	Eigen::MatrixXf A(6, 3), B(6, 1);

    A << -0, -630, 0, 300, 0, 0, -0, 630, 0, -300, -0, 0, 0, 0, -1890, -0, 0, 1890; A_list_.push_back(A);
    B << -31500, 27900, 50400, -9000, 0, 18900; B_list_.push_back(B);
    A << -0, -100, 0, 1000, 0, 0, -0, 100, 0, -1000, -0, 0, 0, 0, -1000, -0, 0, 1000; A_list_.push_back(A);
    B << 0, 110000, 10000, -100000, 0, 10000; B_list_.push_back(B);



}

void Scene4::loadDescription() {
	x_offset_ = 0.0; obs_x_n_ = 1; obs_x_size_ = 40.0; inc_x_ = 10.0;
	y_offset_ = 40.0; obs_y_n_ = 1; obs_y_size_ = 30.0; inc_y_ = 17.0;
	z_offset_ = -1.0; obs_z_n_ = 1; obs_z_size_ = 8.0; inc_z_ = 0.0;
}
