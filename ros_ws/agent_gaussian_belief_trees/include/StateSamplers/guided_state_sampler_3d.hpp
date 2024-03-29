/* Author: Èric Pairet
 * Date:   5 August 2019
 *
 * Description:
 */

#ifndef OMPL_GUIDED_STATE_SAMPLER_3D_
#define OMPL_GUIDED_STATE_SAMPLER_3D_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "../Spaces/R3BeliefSpace.h"
#include "../stl/STLProductGraph.h"

#include <ompl/util/RandomNumbers.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

ob::StateSamplerPtr AllocGuidedStateSampler(const ob::StateSpace *space, const std::vector<ob::State*> &start_states, const double &radius, const int &bias_type, const double &bias_p);

/** \brief Extended state sampler to use with the CForest planning algorithm. It wraps the user-specified
	state sampler.*/
class GuidedStateSampler : public ob::StateSampler
{
public:

	/** \brief Constructor */
  GuidedStateSampler(const ob::StateSpace *space, ob::StateSamplerPtr sampler, const std::vector<ob::State*> start_states, const double &radius, const int &bias_type, const double &bias_p) :
  ob::StateSampler(space),
  sampler_(sampler),
  radius_(radius),
  bias_type_(bias_type),
  ref_idx_(0),
  deltaR_(0),
  keep_growing_(true),
  n_samples_(0),
  bias_p_(bias_p)
  // scout_lead_(scout_lead)
  //rad_(rad)
  {
    c_space_ = space;
    states_to_sample_ = start_states;

    //rad_ = &rad;

    n_uniform = 0;
    n_fixed = 0;

    //radius_angle_ = 0.1;

//    rad_ = rad;

    // TODO: this should come as an argument of the definition
    // TODO: this reference is from the scout state space
    bstate_ = c_space_->allocState();
    if (!states_to_sample_.empty()) {
      bstate_ = states_to_sample_[0];
      // bstate_->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->values[0] = 90.0;
      // bstate_->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->values[1] = 90.0;
      // bstate_->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = 15.0;
    }

    // std::cout << "states to sample" << std::endl;
    // for (auto &i:states_to_sample_){
    //   std::cout << i->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->values[0] << " " ;
    //   std::cout << i->as<CompoundState>()->operator[](0)->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->values[1] << std::endl;
    // }


  }

  /** \brief Destructor */
  ~GuidedStateSampler()
  {
    //std::cout << "\n";
    //std::cout << bias_p_ << std::endl;
    //std::cout << n_uniform << " -- " << n_fixed << std::endl;
   clear();
  }

	/** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
		it will call the sampleUniform() method of the specified sampler. */
	virtual void sampleUniform(ob::State *state);

	/** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
		it will call the sampleUniformNear() method of the specified sampler. */
	virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance);

	/** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
		it will call the sampleGaussian() method of the specified sampler. */
	virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev);

	const ob::StateSpace* getStateSpace() const
	{
		return space_;
	}

	/** \brief Fills the vector StatesToSample_ of states to be sampled in the next
		calls to sampleUniform(), sampleUniformNear() or sampleGaussian(). */
	void setStatesToSample(const std::vector<const ob::State *> &states);

	void clear();
  void test();

protected:

	/** \brief Extracts the next sample when statesToSample_ is not empty. */
	void getNextSample(ob::State *state);
  void sampleFixedLeadUniformly(ob::State *state);
  void sampleDynamicLeadUniformly(ob::State *state);
  void sampleLeadGaussianly(ob::State *state);
  void sampleLeadSequentially(ob::State *state);
  void sampleLeadDynamically(ob::State *state);

  void sampleFixedLeadUniformly(ob::State *state, int abstractstate);
  void sampleUniformWithAbstractState(ob::State *state, int abstractstate);

	/** \brief States to be sampled */
  std::vector<ob::State*> statesToSample_;
	std::vector<ob::State*> states_to_sample_;

	/** \brief Underlying, user-specified state sampler. */
  ompl::RNG rng_;
  const ob::StateSpace *c_space_;
	ob::StateSamplerPtr sampler_;
  ob::State *bstate_;

  bool keep_growing_;

  unsigned int ref_idx_;
  double radius_, radius_angle_;

  double deltaR_;
  //double *rad_;

  double bias_p_;

  unsigned int n_samples_;

  double n_uniform;
  double n_fixed;

  std::vector<int > scout_lead_;

  int bias_type_;

	/** \brief Lock to control the access to the statesToSample_ vector. */
	//boost::mutex statesLock_;
};

#endif
