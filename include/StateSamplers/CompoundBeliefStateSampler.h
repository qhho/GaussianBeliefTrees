#ifndef COMPOUND_BELIEF_SAMPLER_
#define COMPOUND_BELIEF_SAMPLER_

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/StateSampler.h>
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R3BeliefSpace.h"
#include "ompl/base/SpaceInformation.h"

namespace ob = ompl::base;

class CompoundBeliefSampler : public ompl::base::StateSampler
{
    public:
        CompoundBeliefSampler(const ompl::base::SpaceInformation *si, int dimension);

    virtual ~CompoundBeliefSampler(void)
    {
    }

    virtual bool sample(ompl::base::State *state);

    virtual bool sampleUniform(ompl::base::State *state);

    protected:
        dimension_;

}
#endif