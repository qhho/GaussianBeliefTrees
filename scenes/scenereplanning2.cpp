/* Author: Èric Pairet
 * Date:   21 May 2018
 *
 * Description:
 */

#include "scenereplanning2.hpp"

SceneReplanning2::SceneReplanning2() {
	n_obstacles_ = 2;
	loadConstraints();
	loadDescription();
}

void SceneReplanning2::loadConstraints() {
	Eigen::MatrixXf A(6, 3), B(6, 1);

    A << -0, -430, 0, 400, 0, 0, -0, 430, 0, -400, -0, 0, 0, 0, -1720, -0, 0, 1720; A_list_.push_back(A);
    B << -17200, 17200, 34400, 0, 0, 17200; B_list_.push_back(B);
    A << -0, -500, 0, 400, 0, 0, -0, 500, 0, -400, -0, 0, 0, 0, -2000, -0, 0, 2000; A_list_.push_back(A);
    B << -20000, 40000, 40000, -20000, 0, 20000; B_list_.push_back(B);

}

void SceneReplanning2::loadDescription() {
	x_offset_ = 0.0; obs_x_n_ = 1; obs_x_size_ = 40.0; inc_x_ = 10.0;
	y_offset_ = 40.0; obs_y_n_ = 1; obs_y_size_ = 30.0; inc_y_ = 17.0;
	z_offset_ = -1.0; obs_z_n_ = 1; obs_z_size_ = 8.0; inc_z_ = 0.0;
}
