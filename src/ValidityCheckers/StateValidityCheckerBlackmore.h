#pragma once

// Standard libraries
#include <cmath>
#include <cstdlib>
#include <string>

// Boost
#include <boost/math/special_functions/erf.hpp>

// Eigen
#include <eigen3/Eigen/Dense>

// OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include "Spaces/R2BeliefSpace.h"

#include "Scene/Scene.h"

namespace ob = ompl::base;

class StateValidityCheckerBlackmore : public ob::StateValidityChecker {
    public:
        StateValidityCheckerBlackmore(const Scene scene_, const ob::SpaceInformationPtr &si, const double accep_prob);
        ~StateValidityCheckerBlackmore();

        virtual bool isValid(const ob::State *state) const;

    private:
        double p_collision_, erf_inv_result_;

        unsigned int n_obstacles_;
        std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
        std::vector<Eigen::Matrix<float, 6, 1> > B_list_;

        ob::SpaceInformationPtr si_;

        inline double computeInverseErrorFunction(const double &argument) {
            return boost::math::erf_inv(argument);
        }
        bool HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const;
};