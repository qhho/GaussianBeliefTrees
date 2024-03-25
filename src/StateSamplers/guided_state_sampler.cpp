/* Author: Èric Pairet
 * Date:   5 August 2019
 *
 * Description:
 */

#include "guided_state_sampler.hpp"

ob::StateSamplerPtr AllocGuidedStateSampler(const ob::StateSpace *space, const std::vector<ob::State*> &start_states, const double &radius, const int &bias_type, const double &bias_p) {
	GuidedStateSampler *sampler = new GuidedStateSampler(space, space->allocDefaultStateSampler(), start_states, radius, bias_type, bias_p);
	return ob::StateSamplerPtr(sampler);
}

// void GuidedStateSampler::sampleUniform(ob::State *state, int abstractstate){
// 	// if no lead path, uniform sampling to all space
//   // otherwise, sample around the lead path

// 	if (bias_p_ < -1) {
// 		sampleDynamicLeadUniformly(state);
// 	}
// 	else {
// 		if (rng_.uniformReal(0, 1) < bias_p_) {
// 			sampleFixedLeadUniformly(state, abstractstate);
// 		}
// 		else {
// 			sampler_->sampleUniform(state);
// 		}
// 	}
// }


void GuidedStateSampler::sampleUniform(ob::State *state) {
  // if no lead path, uniform sampling to all space
  // otherwise, sample around the lead path
	if (bias_p_ < -10 || states_to_sample_.empty()) {
		sampler_->sampleUniform(state);
		//std::cout << "uniform -- SLP " << std::endl;
	}
	else {
		if (bias_type_ == 1) {
			if (rng_.uniformReal(0,1) < bias_p_){
				sampleDynamicLeadUniformly(state);
			}
			else {
				sampler_->sampleUniform(state);
			}
			//std::cout << "dynamic -- MLP" << std::endl;
		}
		else {
			if (rng_.uniformReal(0, 1) < bias_p_) {
			 	sampleFixedLeadUniformly(state);
				//std::cout << "fixed -- MLP" << " " << bias_p_ << std::endl;
				//n_fixed++;
			}
			else {
				sampler_->sampleUniform(state);
				//std::cout << "uniform -- MLP" << std::endl;
				//n_uniform++;
			}
		}
  }
}

void GuidedStateSampler::sampleFixedLeadUniformly(ob::State *state) {
	 // sampleUniformNear of the DefaultStateSampler samples around the lead if
	 // the defined weights are greater than 0, otherwise it samples uniformly
  	ref_idx_ = rng_.uniformInt(0, states_to_sample_.size() - 1);
	sampler_->sampleUniformNear(state, states_to_sample_[ref_idx_], radius_);

	// angle, separetly because of smaller radius
	//state->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value += rng_.uniformReal(-radius_angle_, radius_angle_);
}

void GuidedStateSampler::sampleFixedLeadUniformly(ob::State *state, int abstractstate) {
	 // sampleUniformNear of the DefaultStateSampler samples around the lead if
	 // the defined weights are greater than 0, otherwise it samples uniformly
  	ref_idx_ = rng_.uniformInt(0, states_to_sample_.size() - 1);
	sampler_->sampleUniformNear(state, states_to_sample_[ref_idx_], radius_);

	// angle, separetly because of smaller radius
	//state->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1)->value += rng_.uniformReal(-radius_angle_, radius_angle_);
}

void GuidedStateSampler::sampleDynamicLeadUniformly(ob::State *state) {
	 // sampleUniformNear of the DefaultStateSampler samples around the lead if
	 // the defined weights are greater than 0, otherwise it samples uniformly
  ref_idx_ = rng_.uniformInt(0, states_to_sample_.size() - 1);
  sampler_->sampleUniformNear(state, states_to_sample_[ref_idx_], radius_ + deltaR_);

	if (keep_growing_ && n_samples_ == 30) { // each n_samples_, increase by 0.1
		deltaR_ += 0.01;
		n_samples_ = 0;
		//*rad_ = radius_ + deltaR_;
		//std::cout << radius_ + deltaR_ << std::endl;
	}
	n_samples_++;
}

void GuidedStateSampler::sampleLeadGaussianly(ob::State *state) {
	 // sampleUniformNear of the DefaultStateSampler samples around the lead if
	 // the defined weights are greater than 0, otherwise it samples uniformly
  ref_idx_ = rng_.uniformInt(0, states_to_sample_.size() - 1);
  sampler_->sampleGaussian(state, states_to_sample_[ref_idx_], radius_);
}

void GuidedStateSampler::sampleLeadSequentially(ob::State *state) {
  unsigned int factor = 10;

  ref_idx_++;
  if (ref_idx_ >= states_to_sample_.size() * factor) {ref_idx_ = 0;}

  int k = round(ref_idx_ / factor);

  R2BeliefSpace::StateType *ostate_css_rss = state->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0);
  R2BeliefSpace::StateType *rstate_css_rss = states_to_sample_[k]->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0);

  sampler_->sampleUniformNear(state, states_to_sample_[k], radius_);
}

void GuidedStateSampler::sampleLeadDynamically(ob::State *state) {
  ref_idx_ = rng_.uniformInt(0, states_to_sample_.size() - 1);

  R2BeliefSpace::StateType *bstate_css_rss = bstate_->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0);
  R2BeliefSpace::StateType *rstate_css_rss = states_to_sample_[ref_idx_]->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0);

  double distance = 0;
  for (int i = 0; i < 2; ++i) {
    distance += (rstate_css_rss->values[i] - bstate_css_rss->values[i])*(rstate_css_rss->values[i] - bstate_css_rss->values[i]);
  }
  distance = sqrt(distance);

  double dr = 2 + (12) * distance / 150;
  sampler_->sampleUniformNear(state, states_to_sample_[ref_idx_], dr);
}

void GuidedStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance) {
	if (!statesToSample_.empty())
		getNextSample(state);
	else
	sampler_->sampleUniformNear(state, near, distance);
}

void GuidedStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev) {
	/*if (!statesToSample_.empty())
		getNextSample(state);
	else
		sampler_->sampleGaussian(state, mean, stdDev);*/
		keep_growing_ = false;
		//*rad_ = 44.21;
		//std::cout << "inside: " << *rad_ << " " << rad_ << std::endl;

		//*rad_ = radius_ + deltaR_;
		//std::cout << "ei " << std::endl;
}

void GuidedStateSampler::setStatesToSample(const std::vector<const ob::State *> &states) {
	//boost::mutex::scoped_lock slock(statesLock_);
	for (size_t i = 0; i < statesToSample_.size(); ++i)
		space_->freeState(statesToSample_[i]);
	statesToSample_.clear();

	statesToSample_.reserve(states.size());
	// push in reverse order, so that the states are popped in order in getNextSample()
	for (std::vector<const ob::State *>::const_iterator st = states.begin(); st != states.end(); ++st) {
		ob::State *s = space_->allocState();
		space_->copyState(s, *st);
		statesToSample_.push_back(s);
	}
}

void GuidedStateSampler::getNextSample(ob::State *state) {
	//boost::mutex::scoped_lock slock(statesLock_);
	space_->copyState(state, statesToSample_.back());
	space_->freeState(statesToSample_.back());
	statesToSample_.pop_back();
}

void GuidedStateSampler::clear() {
	//boost::mutex::scoped_lock slock(statesLock_);
  for (size_t i = 0; i < statesToSample_.size(); ++i)
		space_->freeState(statesToSample_[i]);
	statesToSample_.clear();
	sampler_.reset();
}
