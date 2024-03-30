#include "agent_helpers/Scene.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <eigen3/Eigen/Dense>

Scene::Scene(const std::string& scene_config)
{
    YAML::Node sceneConfig = YAML::LoadFile(scene_config);
    YAML::Node sceneNode = sceneConfig["scene"];
    YAML::Node boundsNode = sceneNode["bounds"];

    if (sceneConfig["scene"]["obstacles"].IsSequence()) {
        for (const auto& obstacle : sceneConfig["scene"]["obstacles"]) {
            define_cube_as_constraints(obstacle["fx"].as<double>(), obstacle["tx"].as<double>(), obstacle["fy"].as<double>(), obstacle["ty"].as<double>(), obstacle["fz"].as<double>(), obstacle["tz"].as<double>());
            std::array<float, 3> min_point = {
                obstacle["fx"].as<float>(),
                obstacle["fy"].as<float>(),
                obstacle["fz"].as<float>()
            };
            std::array<float, 3> max_point = {
                obstacle["tx"].as<float>(),
                obstacle["ty"].as<float>(),
                obstacle["tz"].as<float>()
            };
            obstacles_limits_.push_back(std::make_pair(min_point, max_point));
            n_obstacles_++;
        }
    }

    if (boundsNode.IsSequence()) {
        for (const YAML::Node& bound : boundsNode) {
            if (bound.IsSequence() && bound.size() == 2) {
                std::pair<double, double> boundsPair(bound[0].as<double>(), bound[1].as<double>());
                scene_bounds_.push_back(boundsPair);
            }
        }
    }
    
}

Scene::Scene()
{   
}

Scene::~Scene()
{
}

void Scene::define_cube_as_constraints(const double &fx, const double &tx, const double &fy, const double &ty, const double &fz, const double &tz){

    Eigen::MatrixXf A(6, 3);
    Eigen::VectorXf B(6);
    Eigen::Vector3f p1(tx, fy, tz);
    Eigen::Vector3f q1(fx, fy, tz);
    Eigen::Vector3f r1(fx, fy, fz);

    Eigen::Vector3f p2(tx, fy, tz);
    Eigen::Vector3f q2(tx, ty, tz);
    Eigen::Vector3f r2(tx, ty, fz);

    Eigen::Vector3f p3(fx, ty, tz);
    Eigen::Vector3f q3(tx, ty, tz);
    Eigen::Vector3f r3(fx, ty, fz);

    // H1
    Eigen::Vector3f v1 = q1 - p1;
    Eigen::Vector3f v2 = r1 - p1;
    Eigen::Vector3f n = v1.cross(v2);
    A(0, 0) = n(0); A(0, 1) = n(1); A(0, 2) = n(2);
    B(0) = n.dot(p1);

    // H2
    v1 = p2 - q2;
    v2 = r2 - q2;
    n = v1.cross(v2);
    A(1, 0) = n(0); A(1, 1) = n(1); A(1, 2) = n(2);
    B(1) = n.dot(p2);

    // H3
    v1 = q2 - p3;
    v2 = r2 - p3;
    n = v1.cross(v2);
    A(2, 0) = n(0); A(2, 1) = n(1); A(2, 2) = n(2);
    B(2) = n.dot(p3);

    // H4
    v1 = q1 - p3;
    v2 = r1 - p3;
    n = v2.cross(v1);
    A(3, 0) = n(0); A(3, 1) = n(1); A(3, 2) = n(2);
    B(3) = n.dot(p3);

    // H5
    v1 = r1 - r3;
    v2 = r2 - r3;
    n = v2.cross(v1);
    A(4, 0) = n(0); A(4, 1) = n(1); A(4, 2) = n(2);
    B(4) = n.dot(r1);

    // H6
    v1 = q1 - p3;
    v2 = q2 - p3;
    n = v1.cross(v2);
    A(5, 0) = n(0); A(5, 1) = n(1); A(5, 2) = n(2);
    B(5) = n.dot(q1);

    A_list_.push_back(A);
    B_list_.push_back(B);
}