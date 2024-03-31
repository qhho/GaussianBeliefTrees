#include "StateSamplers/BeliefStateSampler.h"


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


}

void BeliefStateSampler::sampleBias(ob::State *state, const double eigenvalue) {

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
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
}

// void BeliefStateSampler::sampleGaussian(ompl::base::State *state, const State *mean, double stdDev){

// }
// void BeliefStateSampler::sampleUniformNear(ompl::base::State *state, const State *near, double distance){

// }