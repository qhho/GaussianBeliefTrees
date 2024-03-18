#include "StateSamplers/BeliefSampler.h"


ob::StateSamplerPtr AllocBeliefSampler(const ob::StateSpace *space, const std::vector<ob::State*> &start_states, const double &radius, const int &bias_type, const double &bias_p) {
	BeliefSampler *sampler = new BeliefSampler(space, space->allocDefaultStateSampler(), start_states, radius, bias_type, bias_p);
	return ob::StateSamplerPtr(sampler);
}

void BeliefSampler::sampleUniform(ob::State *state) {


}

void BeliefSampler::sampleBias(ob::State *state) {

}

void BeliefSampler::sample(ob::State *state, double eigenvalue){

    if (sampleBias_)
        sampleBias(state);
    else
        sampleUniform(state);
}

void BeliefSampler::sample(ob::State *state){
    sampleUniform(state);
}