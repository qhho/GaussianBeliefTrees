#include "StateSamplers/BeliefStateSampler.h"


BeliefStateSampler::BeliefStateSampler(const ompl::base::StateSpace *ss) :
    StateSampler(ss), sampler_(ss->allocDefaultStateSampler()), dimension_(2), bias_p(0.2)
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

void BeliefStateSampler::sample(ob::State *state, const double eigenvalue){

    if (rng_.uniform01() < bias_p)
        sampleBias(state, eigenvalue);
    else
        sampleUniform(state);
}

void BeliefStateSampler::sample(ob::State *state){
    sampleUniform(state);
}

void BeliefStateSampler::sampleNear(ompl::base::State *state){

}

void BeliefStateSampler::sampleGaussian(ompl::base::State *state, const State *mean, double stdDev){

}
void BeliefStateSampler::sampleUniformNear(ompl::base::State *state, const State *near, double distance){

}