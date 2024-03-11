#include "2d_narrow.hpp"

Narrow2D::Narrow2D() {
	n_obstacles_ = 2;
	loadConstraints();
	loadDescription();
}

void Narrow2D::loadConstraints() {
	Eigen::MatrixXf A(6, 3), B(6, 1);

    A << -0, -430, 0, 300, 0, 0, -0, 430, 0, -300, -0, 0, 0, 0, -1290, -0, 0, 1290; A_list_.push_back(A);
    B << -21500, 12900, 34400, 0, 0, 12900; B_list_.push_back(B);
    A << -0, -500, 0, 300, 0, 0, -0, 500, 0, -300, -0, 0, 0, 0, -1500, -0, 0, 1500; A_list_.push_back(A);
    B << -25000, 30000, 40000, -15000, 0, 15000; B_list_.push_back(B);

}

void Narrow2D::loadDescription() {
	x_offset_ = 0.0; obs_x_n_ = 2; obs_x_size_ = 40.0; inc_x_ = 10.0;
	y_offset_ = 40.0; obs_y_n_ = 1; obs_y_size_ = 30.0; inc_y_ = 17.0;
	z_offset_ = -1.0; obs_z_n_ = 1; obs_z_size_ = 8.0; inc_z_ = 0.0;
}
