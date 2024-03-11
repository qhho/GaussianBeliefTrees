#ifndef SCENE_2D_NARROW_
#define SCENE_2D_NARROW_

#include <vector>
#include <eigen3/Eigen/Dense>

class Narrow2D {
	public:
    Narrow2D();

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
