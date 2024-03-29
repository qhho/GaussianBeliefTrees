#include "agent_gaussian_belief_trees/GaussianBeliefTrees.h"
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


GaussianBeliefTrees::GaussianBeliefTrees(const std::string& scene_config, const std::string& system_config)
: Node("obstacle_publisher")
{
    scene_ = Scene(scene_config);
    system_ = System(system_config);

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&GaussianBeliefTrees::publish_obstacles, this));

    // Example obstacles defined as a vector of vector of points (x, y, z)
    obstacle_vertices_ = {
        {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}}, // First obstacle
        // Add more obstacles if needed
    };
}

void GaussianBeliefTrees::publish_obstacles()
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto& obstacle : obstacle_vertices_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; // Line width
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;

        // Add the vertices to the marker
        for (const auto& vertex : obstacle) {
            geometry_msgs::msg::Point p;
            p.x = vertex[0];
            p.y = vertex[1];
            p.z = vertex[2];
            marker.points.push_back(p);
        }

        // Ensure the LINE_STRIP is closed
        if (obstacle.front() != obstacle.back()) {
            geometry_msgs::msg::Point p;
            p.x = obstacle.front()[0];
            p.y = obstacle.front()[1];
            p.z = obstacle.front()[2];
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    publisher_->publish(marker_array);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::cout<<currentPath<<std::endl;
    std::filesystem::path sceneConfig = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::cout<<sceneConfig<<std::endl;
    std::filesystem::path systemConfig = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    std::cout<<systemConfig<<std::endl;
    auto node = std::make_shared<GaussianBeliefTrees>(sceneConfig, systemConfig);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}