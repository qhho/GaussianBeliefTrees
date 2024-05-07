#pragma once
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class System
{
    public:
        System(const std::string& system_config);
        System();
        ~System();

        // fields:
        double planning_time_;
        double p_safe_;
        double goal_radius_;
        double propagation_size_;
        double pruning_radius_;
        double selection_radius_;
        double sampling_bias_;
        double goal_bias_;
        double cost_threshold_;
        double Q_;
        double R_;
        std::vector<double> starting_configuration_;
        std::vector<double> goal_configuration_;
        std::vector<double> surge_bounds_;
        std::pair<double, double> control_duration_;
        std::vector<std::pair<double, double>> control_bounds_;
};