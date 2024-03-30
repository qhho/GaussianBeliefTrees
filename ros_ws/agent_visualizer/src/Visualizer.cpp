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
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

Visualizer::Visualizer(const std::string& scene_config, const std::string& system_config, const std::string& solution_path)
: Node("gbt_publisher")
{
    scene_ = Scene(scene_config);
    system_ = System(system_config);
    obstacle_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array/obstacles", 10);
    agent_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array/agent", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&Visualizer::timer_callback, this));
    read_agent_positions(solution_path);
}

void Visualizer::read_agent_positions(const std::string& solution_path) {
    int endPadding = 15; // Number of end states so rviz does not repeat immediately. 
    int interpVal = 10; // Number of interpolation values between each point.
    std::ifstream file(solution_path);
    std::string line;
    std::getline(file, line);
    Eigen::Vector3d lastPos(
        system_.starting_configuration_.size() > 0 ? system_.starting_configuration_[0] : 0.0,
        system_.starting_configuration_.size() > 1 ? system_.starting_configuration_[1] : 0.0,
        system_.starting_configuration_.size() > 2 ? system_.starting_configuration_[2] : 0.0
    );

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> row;

        while (std::getline(iss, token, ',')) {
            row.push_back(std::stod(token));
        }
        double z = row.size() > 2 ? row[2] : 0.0;
        Eigen::Vector3d newPos(row[0], row[1], z);
        for (int i = 1; i <= interpVal; ++i) {
            double t = i / double(interpVal);
            Eigen::Vector3d interpolatedPos = lastPos + t * (newPos - lastPos);
            agent_positions_.push_back(interpolatedPos);
        }
        lastPos = newPos;
    }
    agent_positions_.push_back(lastPos);
    for (int i = 0; i < endPadding; ++i) {
        agent_positions_.push_back(lastPos);
    }
}

void Visualizer::timer_callback()
{       
    visualization_msgs::msg::MarkerArray obstacle_marker_array;
    visualization_msgs::msg::MarkerArray agent_marker_array;

    // obstacle visualization
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
        obstacle_marker_array.markers.push_back(marker);
    }
    // agent visualization 
    visualization_msgs::msg::Marker agent_marker;
    agent_marker.header.frame_id = "world";
    agent_marker.header.stamp = this->now();
    agent_marker.ns = "agent";
    agent_marker.id = 0; 
    agent_marker.type = visualization_msgs::msg::Marker::SPHERE;
    agent_marker.action = visualization_msgs::msg::Marker::ADD;
    const auto& pos = agent_positions_[current_pos_ind_ % agent_positions_.size()];
    agent_marker.pose.position.x = pos.x();
    agent_marker.pose.position.y = pos.y();
    agent_marker.pose.position.z = pos.z();
    agent_marker.pose.orientation.x = 0.0;
    agent_marker.pose.orientation.y = 0.0;
    agent_marker.pose.orientation.z = 0.0;
    agent_marker.pose.orientation.w = 1.0;
    agent_marker.scale.x = 1.0;
    agent_marker.scale.y = 1.0; 
    agent_marker.scale.z = 1.0; 
    agent_marker.color.r = 0.0;
    agent_marker.color.g = 1.0;
    agent_marker.color.b = 1.0;
    agent_marker.color.a = 1.0; 
    current_pos_ind_++;
    agent_marker_array.markers.push_back(agent_marker);
    agent_publisher_->publish(agent_marker_array);
    obstacle_publisher_->publish(obstacle_marker_array);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path scene_config = currentPath  / ".." / "configurations" / "scenes" / "scene5.yaml";
    std::filesystem::path system_config = currentPath  / ".." / "configurations" / "systems" / "system1.yaml";
    std::filesystem::path solution_path = currentPath  / ".." / "build" / "solution_60sec_fixedK_03.csv";
    auto node = std::make_shared<Visualizer>(scene_config, system_config, solution_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}