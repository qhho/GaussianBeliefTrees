#include "agent_visualizer/Visualizer.h"
#include "agent_helpers/Scene.h"
#include "agent_helpers/System.h"
#include <chrono>
#include <functional>
#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <fstream>
#include <filesystem>

using namespace std::chrono_literals;

Visualizer::Visualizer(const std::string& scene_config, const std::string& system_config, const std::string& solution_path)
: Node("gbt_publisher")
{
    scene_ = Scene(scene_config);
    system_ = System(system_config);
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Visualizer::timer_callback, this));
}

void Visualizer::timer_callback()
{       
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    for (const auto& obstacle : scene_.obstacles_limits_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE; 
        marker.action = visualization_msgs::msg::Marker::ADD;
        auto& min_point = obstacle.first;
        auto& max_point = obstacle.second;
        marker.pose.position.x = (min_point[0] + max_point[0]) / 2.0f;
        marker.pose.position.y = (min_point[1] + max_point[1]) / 2.0f;
        marker.pose.position.z = (min_point[2] + max_point[2]) / 2.0f;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::abs(max_point[0] - min_point[0]);
        marker.scale.y = std::abs(max_point[1] - min_point[1]);
        marker.scale.z = std::abs(max_point[2] - min_point[2]);
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
    }

    publisher_->publish(marker_array);

    if (offboardSetpointCounter_ < 1) {
        offboardSetpointCounter_++;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path sceneConfig = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::filesystem::path systemConfig = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    auto node = std::make_shared<Visualizer>(sceneConfig, systemConfig, "");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}