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
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        uint64_t offboardSetpointCounter_;
        
        // methods:
        void timer_callback();
};
