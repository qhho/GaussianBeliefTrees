#include "StatePropagators/LinearStatePropagator.h"
#include <ompl/util/Exception.h>
#include <iostream>

using namespace ompl;
LinearRNBeliefPropagator::LinearRNBeliefPropagator(
    const oc::SpaceInformationPtr &si,
    unsigned int stateDim,
    double processNoise, 
    double R, 
    double R_bad, 
    double K_default, 
    std::vector<std::vector<double>> measurement_regions) 
    : oc::StatePropagator(si)
{
    // Store dimensions
    dimensions_ = stateDim;
    K_default_ = K_default;
    
    // Get propagation step size
    duration_ = si->getPropagationStepSize();
    
    // Initialize state vector for use in propagate
    current_state.resize(dimensions_);

    //=========================================================================
    // Open loop system definition (identity by default)
    //=========================================================================
    A_ol_.resize(dimensions_, dimensions_);
    A_ol_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);

    B_ol_.resize(dimensions_, dimensions_);
    B_ol_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);

    //=========================================================================
    // Closed loop system definition
    //=========================================================================
    A_cl_.resize(dimensions_, dimensions_);
    A_cl_ = A_ol_ - B_ol_ * K_default_;

    B_cl_.resize(dimensions_, dimensions_);
    B_cl_ = B_ol_ * K_default_;

    //=========================================================================
    // Discrete closed loop system definition
    //=========================================================================
    A_cl_d_.resize(dimensions_, dimensions_);
    A_cl_d_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_) + A_cl_ * duration_;

    B_cl_d_.resize(dimensions_, dimensions_);
    B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXd::Identity(dimensions_, dimensions_)) * B_cl_;

    // Set up noise matrices
    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    R_ = R*R;
    R_bad_ = R_bad*R_bad;
    
    // Set up measurement matrices
    I = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    H = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    F = Eigen::MatrixXd::Identity(dimensions_, dimensions_);

    // Store measurement regions
    measurementRegions_ = measurement_regions;
    
    // Validate measurement regions
    if (!measurement_regions.empty() && measurement_regions.size() != dimensions_) {
        throw ompl::Exception("Measurement regions must provide bounds for each state dimension");
    }
}

LinearRNBeliefPropagator::LinearRNBeliefPropagator(
    const oc::SpaceInformationPtr &si,
    const Eigen::MatrixXd &A_ol,
    const Eigen::MatrixXd &B_ol,
    double processNoise, 
    double R, 
    double R_bad, 
    double K_default, 
    std::vector<std::vector<double>> measurement_regions) 
    : oc::StatePropagator(si)
{
    // Check dimensions
    if (A_ol.rows() != A_ol.cols()) {
        throw ompl::Exception("System matrix A must be square");
    }


    dimensions_ = A_ol.rows();
    controlDim_ = B_ol.cols();
    K_default_ = K_default;
    
    // Get propagation step size
    duration_ = si->getPropagationStepSize();
    
    // Initialize state vector for use in propagate
    current_state.resize(dimensions_);

    //=========================================================================
    // Open loop system definition
    //=========================================================================
    A_ol_ = A_ol;
    B_ol_ = B_ol;

    //=========================================================================
    // Closed loop system definition
    //=========================================================================

    // Create custom gain matrix using the K_sample
    Eigen::MatrixXd K_mat = Eigen::MatrixXd::Zero(controlDim_, dimensions_);

    std::cout << K_mat << std::endl;

    // Control 0 (ax) affects state 2 (vx)
    K_mat(0, 2) = K_default_;
    // Control 1 (ay) affects state 3 (vy)
    K_mat(1, 3) = K_default_;

    A_cl_.resize(dimensions_, dimensions_);
    A_cl_ = A_ol_ - B_ol_ * K_mat;

    B_cl_.resize(dimensions_, dimensions_);
    B_cl_ = B_ol_ * K_mat;

    //=========================================================================
    // Discrete closed loop system definition
    //=========================================================================
    // A_cl_d_.resize(dimensions_, dimensions_);
    // A_cl_d_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_) + A_cl_ * duration_;

    // B_cl_d_.resize(dimensions_, dimensions_);
    // B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXd::Identity(dimensions_, dimensions_)) * B_cl_;

    // Set up noise matrices
    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    R_ = R*R;
    R_bad_ = R_bad*R_bad;
    
    // Set up measurement matrices
    I = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    H = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    F = Eigen::MatrixXd::Identity(dimensions_, dimensions_);

    // Store measurement regions
    measurementRegions_ = measurement_regions;
    
    // Validate measurement regions
    if (!measurement_regions.empty() && measurement_regions.size() != dimensions_) {
        throw ompl::Exception("Measurement regions must provide bounds for each state dimension");
    }
}

void LinearRNBeliefPropagator::propagate(
    const ob::State *state, 
    const oc::Control *control, 
    const double duration, 
    ob::State *result) const
{
    // Extract state (mean position)
    const RNBeliefSpace::StateType* belief_state = state->as<RNBeliefSpace::StateType>();
    
    // Get current state vector
    Eigen::VectorXd current_state(dimensions_);
    for (unsigned int i = 0; i < dimensions_; i++) {
        current_state(i) = belief_state->getComponent(i);
    }
    
    // Extract control input from OMPL control object
    const oc::RealVectorControlSpace::ControlType* realVecCtrl = 
        static_cast<const oc::RealVectorControlSpace::ControlType*>(control);
    
    // Get control vector
    Eigen::VectorXd u = Eigen::VectorXd::Zero(controlDim_);
    unsigned int numControls = controlDim_;
    
    for (unsigned int i = 0; i < controlDim_ && i < numControls; i++) {
        u(i) = realVecCtrl->values[i];
    }
    
    // Get custom K value if available (usually as the last control value)
    double K_sample = K_default_;
    if (numControls > controlDim_) {
        K_sample = realVecCtrl->values[controlDim_];
    }
    
    // Create custom gain matrix using the K_sample
    Eigen::MatrixXd K_mat = Eigen::MatrixXd::Zero(controlDim_, dimensions_);
    // Control 0 (ax) affects state 2 (vx)
    K_mat(0, 2) = K_sample;
    // Control 1 (ay) affects state 3 (vy)
    K_mat(1, 3) = K_sample;
    
    // std::cout << "A_ol dimensions: " << A_ol_.rows() << "x" << A_ol_.cols() << std::endl;
    // std::cout << "B_ol dimensions: " << B_ol_.rows() << "x" << B_ol_.cols() << std::endl;
    // std::cout << "K_mat dimensions: " << K_mat.rows() << "x" << K_mat.cols() << std::endl;
    // std::cout << "B_ol * K_mat dimensions: " << (B_ol_ * K_mat).rows() << "x" << (B_ol_ * K_mat).cols() << std::endl;

    // Create custom closed-loop matrix using the K_sample
    Eigen::MatrixXd A_cl_custom = A_ol_ - B_ol_ * K_mat;
    
    // Propagate mean state
    // For a linear system: x_new = A*x + B*u
    Eigen::VectorXd new_state = A_ol_ * current_state + B_ol_ * u;


    // std::cout << " --- STATE--- " << std::endl;
    // std::cout << current_state.transpose() << std::endl;
    // std::cout << new_state.transpose() << std::endl;
    // std::cout << " --------------------- " << std::endl;
    // exit(0);
    
    // Set the new mean state
    auto* result_belief = result->as<RNBeliefSpace::StateType>();
    for (unsigned int i = 0; i < dimensions_; i++) {
        result_belief->setComponent(i, new_state(i));
    }
    
    // Propagate covariance
    // Get current covariance and information matrices
    Eigen::MatrixXd sigma_from = belief_state->getSigma();
    Eigen::MatrixXd lambda_from = belief_state->getLambda();
    
    // Predict covariance: Σ_pred = A*Σ*A^T + Q
    Eigen::MatrixXd sigma_pred = A_cl_custom * sigma_from * A_cl_custom.transpose() + Q;
    
    // Check if state is in a good measurement region
    bool inGoodRegion = true; // Default to true if no regions specified
    
    if (!measurementRegions_.empty()) {
        // Check each dimension
        for (unsigned int i = 0; i < dimensions_ && i < measurementRegions_.size(); i++) {
            if (measurementRegions_[i].size() >= 2) {
                double pos = new_state(i);
                if (pos < measurementRegions_[i][0] || pos > measurementRegions_[i][1]) {
                    inGoodRegion = false;
                    break;
                }
            }
        }
    }
    
    // Process update based on measurement quality
    Eigen::MatrixXd lambda_pred, K;
    Eigen::MatrixXd R;
    
    if (inGoodRegion) {
        // Good measurement region
        R = R_ * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        R(2,2) = 0.001;
        R(3,3) = 0.001;
    } else {
        // Bad measurement region
        R = R_bad_ * Eigen::MatrixXd::Identity(dimensions_, dimensions_);
        R(2,2) = 0.001;
        R(3,3) = 0.001;
    }
    
    // Innovation covariance: S = H*Σ_pred*H^T + R
    Eigen::MatrixXd S = (H * sigma_pred * H.transpose()) + R;
    
    // Kalman gain: K = Σ_pred*H^T*S^-1
    K = (sigma_pred * H.transpose()) * S.inverse();
    
    // Predict lambda: λ_pred = A*λ*A^T
    lambda_pred = A_cl_custom * lambda_from * A_cl_custom.transpose();
    
    // Final covariance update: Σ_new = (I - K*H)*Σ_pred
    Eigen::MatrixXd sigma_to = (I - (K * H)) * sigma_pred;
    // Final information update: λ_new = λ_pred + K*H*Σ_pred
    Eigen::MatrixXd lambda_to = lambda_pred + K * H * sigma_pred;
    
    // Set the updated covariance and information matrices

    // std::cout << " --- COVARIANCE--- " << std::endl;
    // std::cout << sigma_from + lambda_from << std::endl;


    // std::cout << sigma_to + lambda_to << std::endl;

    // std::cout << " ----------- " << std::endl;

    // exit(0);

    // Eigen::LLT<Eigen::MatrixXd> lltOfA(sigma_from + lambda_from); // compute the Cholesky decomposition of A
    // if(lltOfA.info() == Eigen::NumericalIssue)
    // {
    //     throw std::runtime_error("Possibly non semi-positive definitie matrix!");
    // }    


    result_belief->setSigma(sigma_to);
    result_belief->setLambda(lambda_to);
}

bool LinearRNBeliefPropagator::canPropagateBackward(void) const
{
    return false;
}