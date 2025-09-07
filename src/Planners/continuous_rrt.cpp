/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "Planners/mod_rrt.hpp"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "Spaces/R2BeliefSpace.h"
#include "Spaces/R2BeliefSpaceEuclidean.h"
#include "Spaces/RNBeliefSpace.h"

inline bool isCompoundStateSpace(const ompl::base::StateSpacePtr &space)
{
    return dynamic_cast<ompl::base::CompoundStateSpace*>(space.get()) != nullptr;
}

ompl::control::mod_RRT::mod_RRT(const SpaceInformationPtr &si) : base::Planner(si, "mod_RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &mod_RRT::setGoalBias, &mod_RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("goal_bias", this, &mod_RRT::setSamplingBias, &mod_RRT::getSamplingBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &mod_RRT::setIntermediateStates, &mod_RRT::getIntermediateStates,
                                "0,1");

    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::control::mod_RRT::~mod_RRT()
{
    freeMemory();
}

void ompl::control::mod_RRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });


    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
                dynamic_cast<base::MinimaxObjective *>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                          "functions w.r.t. state and control. This optimization objective will result in undefined "
                          "behavior",
                          getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }

    prevSolutionCost_ = opt_->infiniteCost();

}

void ompl::control::mod_RRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::mod_RRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::mod_RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // checkValidity();
    // std::cout << "running" << std::endl;
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();
    // std::cout << "here" << std::endl;

    bool compound =  isCompoundStateSpace(si_->getStateSpace());
    max_eigenvalue_ = 10.0;
    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);


        auto rmotionbelief = rmotion->state->as<RNBeliefSpace::StateType>();
        if (compound)
        {
            rmotionbelief = rmotion->state->as<base::CompoundStateSpace::StateType>()->as<RNBeliefSpace::StateType>(0);
        }
        if (DISTANCE_FUNC_ == 1){
            if (rng_.uniform01() < samplingBias_){
                rmotionbelief->setSigma(0.5); //TODO: fix this
            }
            else{
                rmotionbelief->setSigmaX(rng_.uniform01()*max_eigenvalue_);
                rmotionbelief->setSigmaY(rng_.uniform01()*max_eigenvalue_);
            }
        }
        // rmotion->state->as<R2BeliefSpace::StateType>()->setSigma(0.1);
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        // this code is contributed by Jennifer Barry
        std::vector<base::State *> pstates;
        cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);
        if (cd >= siC_->getMinControlDuration())
        {
            Motion *lastmotion = nmotion;
            bool solved = false;
            size_t p = 0;
            for (; p < pstates.size(); ++p)
            {
                /* create a motion */
                auto *motion = new Motion();
                motion->state = pstates[p];
                // we need multiple copies of rctrl
                motion->control = siC_->allocControl();
                siC_->copyControl(motion->control, rctrl);
                motion->steps = 1;
                motion->parent = lastmotion;
                lastmotion = motion;
                nn_->add(motion);

                // std::cout << "Old belief: " << nmotion->state->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX() << " " << nmotion->state->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY() << " " << nmotion->state->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance().trace()  << std::endl;
                // std::cout << "New belief: " << motion->state->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX() << " " << motion->state->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY() << " " << motion->state->as<base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance().trace()  << std::endl;


                auto beliefstate =  motion->state->as<RNBeliefSpace::StateType>();
                if (compound)
                {
                    beliefstate =  motion->state->as<base::CompoundStateSpace::StateType>()->as<RNBeliefSpace::StateType>(0);
                }

                if (DISTANCE_FUNC_ == 0){
                    if (beliefstate->getCovariance()(0,0) > max_eigenvalue_)
                    {
                        max_eigenvalue_ = beliefstate->getCovariance()(0,0);
                    }
                    else if (beliefstate->getCovariance()(1,1) > max_eigenvalue_)
                    {
                        max_eigenvalue_ = beliefstate->getCovariance()(1,1);
                    }
                }
                else if (DISTANCE_FUNC_ == 1){
                    if (beliefstate->getCovariance()(0,0) > max_eigenvalue_)
                    {
                        max_eigenvalue_ = beliefstate->getCovariance()(0,0);
                    }
                    else if (beliefstate->getCovariance()(1,1) > max_eigenvalue_)
                    {
                        max_eigenvalue_ = beliefstate->getCovariance()(1,1);
                    }
                }

                double dist = 0.0;
                solved = goal->isSatisfied(motion->state, &dist);
                if (solved)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }

            // free any states after we hit the goal
            while (++p < pstates.size())
                si_->freeState(pstates[p]);
            if (solved){
                break;
            }

            /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
        }
        else
            for (auto &pstate : pstates)
                si_->freeState(pstate);

    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());


        base::Cost totalIncCost = opt_->identityCost();
        //compute cost of solution
        //add cost of intermediate states as well
        base::Cost solcost(0.0);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
        {
            std::vector<base::State *> pstates;
            siC_->propagateWhileValid(mpath[i]->state, mpath[i]->control, mpath[i]->steps, pstates, true);
            for (size_t p = 0; p < pstates.size(); ++p)
            {
                base::Cost incCost = opt_->motionCost(mpath[i]->state, pstates[p]);
                totalIncCost = opt_->combineCosts(totalIncCost, incCost);
            }
        }
        std::cout << "Cost of solution: " << solcost << std::endl;
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::control::mod_RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
