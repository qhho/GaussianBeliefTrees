#include "StateSamplers/CompoundBeliefStateSampler.h"

CompoundBeliefStateSampler::CompoundBeliefStateSampler(const ompl::base::StateSpace *ss) :
    StateSampler(ss), sampler_(ss->allocDefaultStateSampler()), dimension_(2), bias_p(0.2)
{
    params_.declareParam<double>("bias",
                                 std::bind(&CompoundBeliefStateSampler::setBias, this, std::placeholders::_1),
                                 std::bind(&CompoundBeliefStateSampler::getBias, this));

    params_.declareParam<int>("dimension",
                                 std::bind(&CompoundBeliefStateSampler::setDimension, this, std::placeholders::_1),
                                 std::bind(&CompoundBeliefStateSampler::getDimension, this));
}

void CompoundBeliefStateSampler::sampleUniform(ob::State *state) {


}

void CompoundBeliefStateSampler::sampleBias(ob::State *state, const double eigenvalue) {

}

void CompoundBeliefStateSampler::sample(ob::State *state, const double eigenvalue){

    if (rng_.uniform01() < bias_p)
        sampleBias(state, eigenvalue);
    else
        sampleUniform(state);
}

void CompoundBeliefStateSampler::sample(ob::State *state){
    sampleUniform(state);
}