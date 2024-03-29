#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "agent_helpers/Scene.h"
#include "agent_helpers/System.h"

class GaussianBeliefTrees : public rclcpp::Node
{
    public:
        GaussianBeliefTrees(const std::string& scene_config, const std::string& system_config);
        Scene scene_;
        System system_;

    private:
        void publish_obstacles();
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::vector<std::array<float, 3>>> obstacle_vertices_;
};
