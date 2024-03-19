#include "System.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <eigen3/Eigen/Dense>

System::System(const std::string& system_config)
{   
    YAML::Node systemConfig = YAML::LoadFile(system_config);
    YAML::Node systemNode = systemConfig["system"];
    YAML::Node plannerNode = systemNode["planner"];
    YAML::Node initialConfigurationNode = plannerNode["starting_configuration"];
    YAML::Node goalConfigurationNode = plannerNode["goal_configuration"];
    YAML::Node surgeBoundsNode = plannerNode["surge_bounds"];
    YAML::Node controlBoundsNode = plannerNode["control_bounds"];
    YAML::Node controlDurationNode = plannerNode["control_duration"];
    planning_time_ = systemConfig["system"]["planner"]["planning_time"].as<double>();
    p_safe_ = systemConfig["system"]["planner"]["p_safe"].as<double>();
    goal_radius_ = systemConfig["system"]["planner"]["goal_radius"].as<double>();
    cost_threshold_ = systemConfig["system"]["planner"]["cost_threshold"].as<double>();
    pruning_radius_ = systemConfig["system"]["planner"]["pruning_radius"].as<double>();
    selection_radius_ = systemConfig["system"]["planner"]["selection_radius"].as<double>();
    sampling_bias_ = systemConfig["system"]["planner"]["sampling_bias"].as<double>();
    goal_bias_ = systemConfig["system"]["planner"]["goal_bias"].as<double>();
    propagation_size_ = systemConfig["system"]["planner"]["propagation_size"].as<double>();
    
    if (initialConfigurationNode.IsSequence()) {
        for (YAML::const_iterator it = initialConfigurationNode.begin(); it != initialConfigurationNode.end(); ++it) {
            starting_configuration_.push_back(it->as<double>());
        }
    }

    if (goalConfigurationNode.IsSequence()) {
        for (YAML::const_iterator it = goalConfigurationNode.begin(); it != goalConfigurationNode.end(); ++it) {
            goal_configuration_.push_back(it->as<double>());
        }
    }

    if (surgeBoundsNode.IsSequence()) {
        for (YAML::const_iterator it = surgeBoundsNode.begin(); it != surgeBoundsNode.end(); ++it) {
            surge_bounds_.push_back(it->as<double>());
        }
    }

    if (controlDurationNode.IsSequence()){
        for(const YAML::Node& duration : controlDurationNode){
            if(duration.IsSequence() && duration.size() == 2){
                control_duration_.first = duration[0].as<double>();
                control_duration_.second = duration[1].as<double>();
            }
        }
    }

    if (controlBoundsNode.IsSequence()) {
        for (const YAML::Node& bound : controlBoundsNode) {
            if (bound.IsSequence() && bound.size() == 2) {
                std::pair<double, double> boundsPair(bound[0].as<double>(), bound[1].as<double>());
                control_bounds_.push_back(boundsPair);
            }
        }
    }
}

System::System()
{
}

System::~System()
{
}