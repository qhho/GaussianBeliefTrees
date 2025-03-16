/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Texas A&M University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* Author: Qi Heng, Ali-akbar Agha-mohammadi, Saurav Agarwal */

#ifndef THREEDPOINT_STATE_PROPAGATOR_FIXEDK_
#define THREEDPOINT_STATE_PROPAGATOR_FIXEDK_

// #include "SpaceInformation/SpaceInformation.h"
#include "ompl/control/SpaceInformation.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "Spaces/R3BeliefSpace.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Mat3;

/** \brief State propagation for a 3d point motion model. */
class ThreeDSimpleStatePropagatorFixedK : public oc::StatePropagator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** \brief Construct representation of a unicycle state propagator.
    */
    ThreeDSimpleStatePropagatorFixedK(const oc::SpaceInformationPtr &si,  double processNoise, double R, double R_bad, double K_default, std::vector<std::vector<double > > measurement_regions);

    virtual ~ThreeDSimpleStatePropagatorFixedK(void)
    {
    }

    /** \brief Will always return false, as the simulation can only proceed forward in time */
    virtual bool canPropagateBackward(void) const;

    /** \brief Propagate from a state, under a given control, for some specified amount of time.
        We use the motion model to do the actual number crunching.

    */
    virtual void propagate(const ompl::base::State *state, const ompl::control::Control* control, const double duration, ompl::base::State *result) const;

private:

    Eigen::Matrix3d A_ol_, B_ol_, A_cl_, B_cl_, A_cl_d_, B_cl_d_;

    double duration_;

    mutable double x_pose, x_pose_reference, y_pose, y_pose_reference, z_pose, z_pose_reference;

    mutable const R3BeliefSpace::StateType *start_css;

    
    mutable oc::RealVectorControlSpace::ControlType *control_css;

    const Eigen::Vector3d start_css_rvs_pose;
    const Eigen::Matrix3d start_css_rvs_cov;

    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix3d H = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix3d F = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix3d A_BK_;

    Eigen::Matrix3d sigma_pred, lambda_pred, K, Q, Ak;

    double K_ = 0.1;

    double myK_ = 0.3;
    double R_;
    double R_bad_;

    std::vector<std::vector<double>> measurementRegions_;


    mutable const R3BeliefSpace::StateType *result_css;
    ob::RealVectorStateSpace::StateType *result_css_rvs_pose;
    Eigen::Matrix3d result_css_rvs_cov;

protected:

    int dimensions_ = 3;


    // Belief::Belief beliefModel_;

    // MotionModelMethod::MotionModelPointer motionModel_;

    // firm::SpaceInformation::SpaceInformationPtr siF_;
    /**
    You can add a simulated environment here where the controls can get applied, useful for
    showing the graphics, very similar to the concept of ActuationSystem in PMPL.
    */
};

#endif