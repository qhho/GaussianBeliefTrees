/*
 * expected_cost_objective.hpp
 *
 *  Created on: 2024
 *      Author: Qi Heng Ho
 *
 *  Expected Cost objective. Define different expectation cost objectives.
 */

#ifndef OMPL_CONTRIB_EXPECTATION_OBJECTIVES_
#define OMPL_CONTRIB_EXPECTATION_OBJECTIVES_

//OMPL
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
// #include <ompl/base/objectives/ControlDurationObjective.h>
#include "../Spaces/R2BeliefSpace.h"
#include "../Spaces/RNBeliefSpace.h"

namespace ob = ompl::base;
namespace oc = ompl::control;


class ExpectedPathLengthObjective : public ob::PathLengthOptimizationObjective
{
    public:
        ExpectedPathLengthObjective(const ob::SpaceInformationPtr& si) :
        ob::PathLengthOptimizationObjective(si)
        {
            compound_ = si->getStateSpace()->isCompound();
        }
 
        ob::Cost motionCost(const State *s1, const State *s2) const override
        {
            auto beliefstate_s1 = s1->as<RNBeliefSpace::StateType>();
            auto beliefstate_s2 = s2->as<RNBeliefSpace::StateType>();
            if (compound_)
            {
                beliefstate_s1 =  s1->as<ob::CompoundStateSpace::StateType>()->as<RNBeliefSpace::StateType>(0);
                beliefstate_s2 =  s2->as<ob::CompoundStateSpace::StateType>()->as<RNBeliefSpace::StateType>(0);
            }


            Eigen::Vector2d diff = beliefstate_s1->getXY() - beliefstate_s2->getXY();
            // std::cout << s1->as<R2BeliefSpace::StateType>()->getSigma().trace() << " " << s1->as<R2BeliefSpace::StateType>()->getLambda().trace() << std::endl;
            return Cost(sqrt(diff.norm()*diff.norm() + beliefstate_s1->getCovariance().trace() + beliefstate_s2->getCovariance().trace()));
        }

    bool compound_;
};


class ExpectedControlEffortObjective : public ob::PathLengthOptimizationObjective
{
    public:
        ExpectedControlEffortObjective(const ob::SpaceInformationPtr& si, const int control_dimension) :
        ob::PathLengthOptimizationObjective(si)
        {
            control_dimension_ = control_dimension;
        }

        double dotprod(const oc::Control *u, const int dimension) const {
            double result = 0.0;

            for (int i = 0; i < dimension; i++) {
                result += u->as<oc::RealVectorControlSpace::ControlType>()->values[i] * u->as<oc::RealVectorControlSpace::ControlType>()->values[i];
            }
            return result;
        }
    
        ob::Cost motionCost(const State *s1, const State *s2, const oc::Control *u) const
        {
            Eigen::Vector2d diff = s1->as<R2BeliefSpace::StateType>()->getXY() - s2->as<R2BeliefSpace::StateType>()->getXY();
            // return Cost(sqrt(diff.norm()*diff.norm() + s1->as<R2BeliefSpace::StateType>()->getCovariance().trace() + s2->as<R2BeliefSpace::StateType>()->getCovariance().trace()));
            double nominalcontrolEffort = dotprod(u, control_dimension_ - 1);
            double K = u->as<oc::RealVectorControlSpace::ControlType>()->values[control_dimension_];
            double uncertaintyTerm = (K*K*s1->as<R2BeliefSpace::StateType>()->getLambda()).trace();

            return Cost(nominalcontrolEffort + uncertaintyTerm);
        }
        // ob::Cost motionCost(const State *s1, const State *s2) const override
        // {
        //     Eigen::Vector2d diff = s1->as<R2BeliefSpace::StateType>()->getXY() - s2->as<R2BeliefSpace::StateType>()->getXY();
        //     return Cost(diff.norm());
        // }

        ob::Cost ExpectedCost(const State *s1, const State *s2, const oc::Control *u) const
        {
            Eigen::Vector2d diff = s1->as<R2BeliefSpace::StateType>()->getXY() - s2->as<R2BeliefSpace::StateType>()->getXY();
            return Cost(diff.norm());
        }
    int control_dimension_;
};

ob::OptimizationObjectivePtr getExpectedPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getExpectedControlEffortObjective(const ob::SpaceInformationPtr& si, const int control_dimension);



#endif