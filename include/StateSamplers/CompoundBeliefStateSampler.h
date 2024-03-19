#ifndef COMPOUND_BELIEF_SAMPLER_
#define COMPOUND_BELIEF_SAMPLER_

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/StateSampler.h>
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R3BeliefSpace.h"
#include "ompl/base/SpaceInformation.h"

namespace ob = ompl::base;

class CompoundBeliefStateSampler : public ompl::base::StateSampler
{
    public:
        CompoundBeliefStateSampler(const ompl::base::StateSpace *ss);


    virtual ~CompoundBeliefStateSampler(void)
    {
    }

    virtual void sample(ompl::base::State *state);

    virtual void sample(ompl::base::State *state, const double eigenvalue);

    virtual void sampleUniform(ompl::base::State *state);

    void sampleBias(ompl::base::State *state, const double eigenvalue);

    int getDimension() const {
        return dimension_;
    }

    void setDimension(int dimension) {
        dimension_ = dimension;
    }

    double getBias() const {
        return bias_p;
    }

    void setBias(double bias_p) {
        bias_p = bias_p;
    }

    /** \brief Get the parameters for the state sampler */
    ParamSet &params()
    {
        return params_;
    }

    /** \brief Get the parameters for the state sampler */
    const ParamSet &params() const
    {
        return params_;
    }

    protected:
        
        double bias_p;
        
        int dimension_;

        /** \brief The sampler to build upon */
        ompl::base::StateSamplerPtr sampler_;

        ParamSet params_;

};

#endif