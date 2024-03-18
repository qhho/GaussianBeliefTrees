#ifndef BELIEF_SAMPLER_
#define BELIEF_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R3BeliefSpace.h"
#include "ompl/base/SpaceInformation.h"

namespace ob = ompl::base;

class BeliefSampler : public ompl::base::StateSampler
{
    public:
        BeliefSampler(const ompl::base::SpaceInformation *si, int dimension);

    virtual ~BeliefSampler(void)
    {
    }

    virtual bool sample(ompl::base::State *state);

    virtual bool sampleUniform(ompl::base::State *state);

    protected:
        dimension_;

}

#endif