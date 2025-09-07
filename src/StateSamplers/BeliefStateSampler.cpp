#include "StateSamplers/BeliefStateSampler.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/RNBeliefSpace.h"

BeliefStateSampler::BeliefStateSampler(const ompl::base::SpaceInformation *si) :
    ValidStateSampler(si), sampler_(si->allocStateSampler()), dimension_(2), bias_p(0.2)
{
    params_.declareParam<double>("bias",
                                 std::bind(&BeliefStateSampler::setBias, this, std::placeholders::_1),
                                 std::bind(&BeliefStateSampler::getBias, this));

    params_.declareParam<int>("dimension",
                                 std::bind(&BeliefStateSampler::setDimension, this, std::placeholders::_1),
                                 std::bind(&BeliefStateSampler::getDimension, this));
}

void BeliefStateSampler::sampleUniform(ob::State *state) {
    // Use the underlying sampler for uniform sampling
    sampler_->sampleUniform(state);
    
    // If it's a belief state, also sample covariance
    if (auto belief_state = dynamic_cast<RNBeliefSpace::StateType*>(state)) {
        // Sample random covariance matrix (positive definite)
        Eigen::MatrixXd cov = Eigen::MatrixXd::Random(dimension_, dimension_);
        cov = cov.transpose() * cov; // Make it positive definite
        cov = 0.1 * cov; // Scale down
        belief_state->setSigma(cov);
    }
}

void BeliefStateSampler::sampleBias(ob::State *state, const double eigenvalue) {
    // Use the underlying sampler for position sampling
    sampler_->sampleUniform(state);
    
    // If it's a belief state, set covariance based on eigenvalue
    if (auto belief_state = dynamic_cast<RNBeliefSpace::StateType*>(state)) {
        Eigen::MatrixXd cov = eigenvalue * Eigen::MatrixXd::Identity(dimension_, dimension_);
        belief_state->setSigma(cov);
    }
}

bool BeliefStateSampler::sample(ob::State *state, const double eigenvalue){

    if (rng_.uniform01() < bias_p)
        sampleBias(state, eigenvalue);
    else
        sampleUniform(state);

    return true;
}

bool BeliefStateSampler::sample(ob::State *state){
    sampleUniform(state);
    return true;
}

bool BeliefStateSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance){
    // Sample near the given state
    sampler_->sampleUniformNear(state, near, distance);
    
    // If it's a belief state, also sample covariance near the given state
    if (auto belief_state = dynamic_cast<RNBeliefSpace::StateType*>(state)) {
        if (auto near_belief = dynamic_cast<const RNBeliefSpace::StateType*>(near)) {
            // Sample covariance near the given covariance
            Eigen::MatrixXd near_cov = near_belief->getCovariance();
            Eigen::MatrixXd cov = near_cov + 0.1 * Eigen::MatrixXd::Random(dimension_, dimension_);
            cov = cov.transpose() * cov; // Ensure positive definiteness
            belief_state->setSigma(cov);
        }
    }
    
    return true;
}

// void BeliefStateSampler::sampleGaussian(ompl::base::State *state, const State *mean, double stdDev){

// }
// void BeliefStateSampler::sampleUniformNear(ompl::base::State *state, const State *near, double distance){

// }