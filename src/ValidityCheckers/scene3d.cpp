/* Author: Èric Pairet
 * Date:   21 May 2018
 *
 * Description:
 */

#include "scene3d.hpp"

Scene3d::Scene3d() {
	n_obstacles_ = 3; //2
	loadConstraints();
	loadDescription();
}

void Scene3d::loadConstraints() {
	Eigen::MatrixXf A(6, 3), B(6, 1);

	//SCENARIO 2 FINAL ICRA//
	
	A << -0, -200, 0, 200, 0, 0, -0, 200, 0, -200, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
	B << -16000, 20000, 20000, -16000, 0, 4000; B_list_.push_back(B);
	A << -0, -200, 0, 200, 0, 0, -0, 200, 0, -200, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
	B << -12000, 8000, 16000, -4000, 0, 4000; B_list_.push_back(B);
	A << -0, -200, 0, 200, 0, 0, -0, 200, 0, -200, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
	B << -4000, 16000, 8000, -12000, 0, 4000; B_list_.push_back(B);

	
	// A << -0, -200, 0, 100, 0, 0, -0, 200, 0, -100, -0, 0, 0, 0, -200, -0, 0, 200; A_list_.push_back(A);
	// B << 10000, 9000, -8000, -7000, 0, 2000; B_list_.push_back(B);
// A << -0, -400, 0, 100, 0, 0, -0, 400, 0, -100, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
// B << 20000, 5000, -16000, -1000, 0, 4000; B_list_.push_back(B);
// A << -0, -200, 0, 100, 0, 0, -0, 200, 0, -100, -0, 0, 0, 0, -200, -0, 0, 200; A_list_.push_back(A);
// B << 10000, 9000, -8000, -7000, 0, 2000; B_list_.push_back(B);
// A << -0, -450, 0, 200, 0, 0, -0, 450, 0, -200, -0, 0, 0, 0, -900, -0, 0, 900; A_list_.push_back(A);
// B << -22500, 9000, 31500, 0, 0, 9000; B_list_.push_back(B);
// A << -0, -450, 0, 200, 0, 0, -0, 450, 0, -200, -0, 0, 0, 0, -900, -0, 0, 900; A_list_.push_back(A);
// B << -22500, 20000, 31500, -11000, 0, 9000; B_list_.push_back(B);

	////***FIRST SCENARIO MOTION PLANNING
// A << -0, -700, 0, 500, 0, 0, -0, 700, 0, -500, -0, 0, 0, 0, -3500, -0, 0, 3500; A_list_.push_back(A);
// B << -35000, 42500, 70000, -7500, 0, 35000; B_list_.push_back(B);
// A << -0, -400, 0, 100, 0, 0, -0, 400, 0, -100, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
// B << 20000, 5000, -16000, -1000, 0, 4000; B_list_.push_back(B);
// A << -0, -200, 0, 100, 0, 0, -0, 200, 0, -100, -0, 0, 0, 0, -200, -0, 0, 200; A_list_.push_back(A);
// B << 10000, 9000, -8000, -7000, 0, 2000; B_list_.push_back(B);
	////**first scenario_7sep


	// A << -0, -400, 0, 100, 0, 0, -0, 400, 0, -100, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
	// B << 20000, 4000, -16000, 0, 0, 4000; B_list_.push_back(B);
	// A << -0, -450, 0, 200, 0, 0, -0, 450, 0, -200, -0, 0, 0, 0, -900, -0, 0, 900; A_list_.push_back(A);
	// B << -22500, 9000, 31500, 0, 0, 9000; B_list_.push_back(B);
	// A << -0, -450, 0, 200, 0, 0, -0, 450, 0, -200, -0, 0, 0, 0, -900, -0, 0, 900; A_list_.push_back(A);
	// B << -22500, 20000, 31500, -11000, 0, 9000; B_list_.push_back(B);
// 	/////***FIRST SCENARIO MOTION PLANNING

// 	/////***PHI_2 SCENARIO MOTION PLANNING
// 	A << -0, -600, 0, 400, 0, 0, -0, 600, 0, -400, -0, 0, 0, 0, -2400, -0, 0, 2400; A_list_.push_back(A);
// B << -36000, 32000, 60000, -8000, 0, 24000; B_list_.push_back(B);
// A << -0, -400, 0, 100, 0, 0, -0, 400, 0, -100, -0, 0, 0, 0, -400, -0, 0, 400; A_list_.push_back(A);
// B << 20000, 5000, -16000, -1000, 0, 4000; B_list_.push_back(B);
	/////***PHI_2 SCENARIO MOTION PLANNING

}

void Scene3d::loadDescription() {
	x_offset_ = 0.0; obs_x_n_ = 2; obs_x_size_ = 40.0; inc_x_ = 10.0;
	y_offset_ = 40.0; obs_y_n_ = 1; obs_y_size_ = 30.0; inc_y_ = 17.0;
	z_offset_ = -1.0; obs_z_n_ = 1; obs_z_size_ = 8.0; inc_z_ = 0.0;
}
