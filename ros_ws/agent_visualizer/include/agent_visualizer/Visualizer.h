#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "agent_helpers/Scene.h"
#include "agent_helpers/System.h"
// OMPL
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/tools/debug/Profiler.h>
#include "ompl/tools/benchmark/Benchmark.h"
#include <limits>

class Visualizer : public rclcpp::Node
{
    public:
        Visualizer(const std::string& scene_config, const std::string& system_config, const std::string& solution_path);

        // fields:
        Scene scene_;
        System system_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<Eigen::Vector3d> agent_positions_;
        int current_pos_ind_ = 0;

        // methods:
        void timer_callback();
        void read_agent_positions(const std::string& solution_path);
};
