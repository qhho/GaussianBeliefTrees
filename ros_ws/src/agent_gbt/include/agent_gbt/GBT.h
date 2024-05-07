#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "agent_helpers/Scene.h"
// #include "agent_helpers/System.h"
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


#include "StatePropagators/SimpleStatePropagator.h"
#include "ValidityCheckers/StateValidityCheckerBlackmore.h" 
#include "StatePropagators/SimpleStatePropagator.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RealVectorBeliefSpace.h"
#include "OptimizationObjectives/state_cost_objective.hpp"
#include "OptimizationObjectives/expected_cost_objective.hpp"
#include "Planners/SSBT.hpp"
#include "Scene/Scene.h"
#include "System/System.h"

class GBT : public rclcpp::Node
{
    public:
        GBT(const std::string& scene_config, const std::string& system_config);

        // fields:
        Scene scene_;
        System system_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agent_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<Eigen::Vector3d> agent_positions_;
        int current_pos_ind_ = 0;
        std::string scene_id_;
        double planning_depth_, watchdog_period_, solving_time_, discretisation_time_, min_control_duration_, max_control_duration_, p_safe_, goal_tolerance_, confidence_level_;
        std::vector<double> planning_bounds_x_, planning_bounds_y_, planning_bounds_z_, start_configuration_, goal_configuration_, controller_parameters_, surge_bounds_, heave_bounds_, forward_acceleration_bounds_, turning_rate_bounds_, heave_acceleration_bounds_, system_noise_;
        Eigen::Matrix<double, 2, 2, Eigen::DontAlign> initial_covariance_;
        std::vector<const ob::State *> path_states_;
        std::vector<const oc::Control *> path_controls_;
        ob::StateValidityCheckerPtr om_stat_val_check_;
        oc::SimpleSetupPtr simple_setup_;
        ob::StateSpacePtr c_space_;

        // methods:
        void timer_callback();
        ob::StateSpacePtr constructCSpace();
        oc::ControlSpacePtr constructCtrlSpace(ob::StateSpacePtr c_space);
        std::string planWithSimpleSetup();
        void solve(ob::PlannerPtr planner);
        void SaveSolutionPath(oc::PathControl path_control, ob::StateSpacePtr space, std::string pathstring);
};
