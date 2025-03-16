#ifndef BELIEF_SAMPLER_
#define BELIEF_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R3BeliefSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/control/SpaceInformation.h"

namespace ob = ompl::base;


class BeliefStateSampler : public ompl::base::ValidStateSampler
{
    public:
        BeliefStateSampler(const ompl::base::SpaceInformation *si);


    virtual ~BeliefStateSampler(void)
    {
    }

    virtual bool sample(ompl::base::State *state) override;

    virtual bool sample(ompl::base::State *state, const double eigenvalue);

    virtual void sampleUniform(ompl::base::State *state);

    virtual bool sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    // virtual void sampleNear(ompl::base::State *state);

    // virtual void sampleGaussian(ompl::base::State *state, const State *mean, double stdDev);

    // virtual void sampleUniformNear(ompl::base::State *state, const State *near, double distance);

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

        ompl::RNG rng_;
        
        double bias_p;
        
        int dimension_;

        /** \brief The sampler to build upon */
        ompl::base::StateSamplerPtr sampler_;

        ParamSet params_;

};

#endif