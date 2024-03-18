include "StateSamplers/CompoundBeliefStateSampler.h"

ob::StateSamplerPtr AllocBeliefSampler(const ob::StateSpace *space, const std::vector<ob::State*> &start_states, const double &radius, const int &bias_type, const double &bias_p) {
	CompoundBeliefSampler *sampler = new CompoundBeliefSampler(space, space->allocDefaultStateSampler(), start_states, radius, bias_type, bias_p);
	return ob::StateSamplerPtr(sampler);
}

void CompoundBeliefSampler::sampleUniform(ob::State *state) {


}

void CompoundBeliefSampler::sampleBias(ob::State *state) {

}

void CompoundBeliefSampler::sample(ob::State *state, double eigenvalue){

    if (sampleBias_)
        sampleBias(state);
    else
        sampleUniform(state);
}

void CompoundBeliefSampler::sample(ob::State *state){
    sampleUniform(state);
}