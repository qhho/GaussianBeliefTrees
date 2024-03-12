#pragma once
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class Scene
{
    public:
        Scene(const std::string& scene_config);
        Scene();
        ~Scene();

        // fields:
        std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
        std::vector<Eigen::Matrix<float, 6, 1> > B_list_;
        double x_min_;
        double x_max_;
        double y_min_;
        double y_max_;
        double z_min_;
        double z_max_;
        unsigned int n_obstacles_ = 0;

    private:

        // methods:
        void define_cube_as_constraints(const double &fx, const double &tx, const double &fy, const double &ty, const double &fz, const double &tz);
};