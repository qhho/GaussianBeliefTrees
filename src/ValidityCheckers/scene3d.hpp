/* Author: Èric Pairet, Qi Heng
 * Date:   21 May 2018
 *
 * Description:
 */

#ifndef SCENE_3D_
#define SCENE_3D_

#include <vector>
#include <eigen3/Eigen/Dense>

class Scene3d {
	public:
    Scene3d();

    unsigned int n_obstacles_;
    std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
    std::vector<Eigen::Matrix<float, 6, 1> > B_list_;
    int obs_x_n_, obs_y_n_, obs_z_n_;
    double x_offset_, y_offset_, z_offset_, obs_x_size_, obs_y_size_, obs_z_size_, inc_x_, inc_y_, inc_z_;

  private:
    void loadConstraints();
    void loadDescription();
};

#endif
