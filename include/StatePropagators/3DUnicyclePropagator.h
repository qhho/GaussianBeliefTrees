#ifndef _3D_UNICYCLE_PROPAGATOR_
#define _3D_UNICYCLE_PROPAGATOR_

//Standard libraries
#include <cmath>

//Eigen
#include <eigen3/Eigen/Dense>

//OMPL
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "Spaces/R3BeliefSpace.h"

//OMPL namespaces
namespace ob = ompl::base;
namespace oc = ompl::control;

typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Mat3;
typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Mat44;
typedef Eigen::Matrix<double, 2, 4, Eigen::DontAlign> Mat24;
typedef Eigen::Matrix<double, 4, 2, Eigen::DontAlign> Mat42;

#define _USE_MATH_DEFINES

class DynUnicycleControlSpace3D : public oc::StatePropagator {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DynUnicycleControlSpace3D(const oc::SpaceInformationPtr &si, double processNoise, double R, double R_bad, double K_default, std::vector<std::vector<double > > measurement_regions);

        virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const;
        // bool TheHyperplaneCCValidityChecker(Eigen::MatrixXf &A, Eigen::MatrixXf &B, double &x_pose, double &y_pose, double &z_pose, Eigen::MatrixXf &PX);
        double erf_inv_result_;
    private:
        double duration_, cxx_, cyy_, czz_;
        std::vector<double> controller_parameters_, forward_acceleration_bounds_, turning_rate_bounds_, heave_acceleration_bounds_, surge_bounds_, heave_bounds_, system_noise_;
        Eigen::MatrixXf A_ol_, B_ol_, K_, Pw_, A_cl_, B_cl_, A_cl_d_, B_cl_d_;

        // in execution
        mutable double x_pose, y_pose, z_pose, yaw, surge, heave, Pxx_init, Pyy_init, Pzz_init;
        mutable const ob::CompoundStateSpace::StateType *start_css;
        mutable const ob::RealVectorStateSpace::StateType *start_css_rvs_pose, *start_css_rvs_cov;
        mutable double cos_y, sin_y;
        mutable double x_pose_reference, y_pose_reference, z_pose_reference, yaw_reference, surge_reference, heave_reference;
        mutable const oc::CompoundControlSpace::ControlType *control_css;
        mutable const oc::RealVectorControlSpace::ControlType *control_css_rvs_pose;
        mutable double u_0, u_1, u_2, u_bar_0, u_bar_1, u_bar_2, surge_final, heave_final;
        mutable ob::CompoundStateSpace::StateType *result_css;

        int dimensions_ = 3;

        Eigen::Matrix3d A_BK_;

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d A_cl_d_33_;

        Eigen::Matrix3d Q;

        double myK_ = 0.1;
        double R_;
        double R_bad_;
        std::vector<std::vector<double>> measurementRegions_;

        std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
        std::vector<Eigen::Matrix<float, 6, 1> > B_list_;
};

#endif
