/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez, Jonathan Gammell */

#include "Planners/rrbt.hpp"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include "ompl/util/GeometricEquations.h"
// #include "ompl/util/controlEquations.h"

// #include "../Spaces/R2BeliefSpaceEuclidean.h"
#include "../scenes/2d_narrow.hpp"
// #include "../ValidityCheckers/scene3.hpp"
// #include "../ValidityCheckers/scene4.hpp"

#include <boost/math/special_functions/erf.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;

ompl::control::RRBT::RRBT(const SpaceInformationPtr &si)
  : base::Planner(si, "RRBT")
{
    siC_ = si.get();
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &RRBT::setRange, &RRBT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRBT::setGoalBias, &RRBT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &RRBT::setRewireFactor, &RRBT::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRBT::setKNearest, &RRBT::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRBT::setDelayCC, &RRBT::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &RRBT::setTreePruning, &RRBT::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &RRBT::setPruneThreshold, &RRBT::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &RRBT::setPrunedMeasure, &RRBT::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &RRBT::setInformedSampling, &RRBT::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &RRBT::setSampleRejection, &RRBT::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &RRBT::setNewStateRejection,
                                &RRBT::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &RRBT::setAdmissibleCostToCome,
                                &RRBT::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &RRBT::setOrderedSampling, &RRBT::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &RRBT::setBatchSize, &RRBT::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &RRBT::setFocusSearch, &RRBT::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &RRBT::setNumSamplingAttempts,
                                        &RRBT::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });

    double K_ = 0.3;
    double myduration_ = si->getPropagationStepSize();
    A_ol_.resize(2, 2);
    A_ol_ << 1.0, 0.0,
            0.0, 1.0;

    B_ol_.resize(2, 2);
    B_ol_ << 1.0, 0.0,
            0.0, 1.0;

    //=========================================================================
    // Close loop system definition
    //=========================================================================
    A_cl_.resize(2, 2);
    A_cl_ = A_ol_ - B_ol_ * K_;

    B_cl_.resize(2, 2);
    B_cl_ = B_ol_ * K_;

    //=========================================================================
    // Discrete close loop system definition
    //=========================================================================
    A_cl_d_.resize(2, 2);
    A_cl_d_ = Eigen::MatrixXd::Identity(2, 2) + A_cl_ * myduration_;

    B_cl_d_.resize(2, 2);
    B_cl_d_ = A_cl_d_.inverse() * (A_cl_d_ - Eigen::MatrixXd::Identity(2, 2)) * B_cl_;

    double processNoise = 0.1;
    Q = pow(processNoise, 2) * Eigen::MatrixXd::Identity(2, 2);

    p_collision_ = 0.05;


    std::string scene_id = "scene4";

	OMPL_INFORM("scene is %s", scene_id.c_str());
	if (scene_id == "2d_narrow") {
		Narrow2D scene = Narrow2D();
		n_obstacles_ = scene.n_obstacles_;
		A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
		B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
        // std::cout << "done" << std::endl;
        // std::cout << A_list_(0,0) << std::endl;
	}
    // else if (scene_id == "scene4") {
	// 	Scene4 scene = Scene4();
	// 	n_obstacles_ = scene.n_obstacles_;
	// 	A_list_.resize(n_obstacles_); A_list_ = scene.A_list_;
	// 	B_list_.resize(n_obstacles_); B_list_ = scene.B_list_;
    //     // std::cout << "done" << std::endl;
    //     // std::cout << A_list_(0,0) << std::endl;
	// }
    else{
        OMPL_ERROR("Scene not found");
    }

	erf_inv_result_ = boost::math::erf_inv(1 - 2 * p_collision_ / n_obstacles_);
}

ompl::control::RRBT::~RRBT()
{
    freeMemory();
}

void ompl::control::RRBT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();

    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void ompl::control::RRBT::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    OMPL_INFORM("Free-ing memory");
    freeMemory();
    OMPL_INFORM("Freed memory");
    if (nn_)
        nn_->clear();
    // for (auto it:toBeDeleted)
    //     delete it;
    OMPL_INFORM("Cleared nn");
    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}

ompl::base::PlannerStatus ompl::control::RRBT::solve(const base::PlannerTerminationCondition &ptc)
{
    //TODO: clean code
    //TODO: update cost computation method

    // std::cout << "trying to solve" << std::endl;
    // checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    bool symCost = opt_->isSymmetric();
    // std::cout << "trying to solve" << std::endl;
    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, st);
            siC_->nullControl(motion->control_);
            motion->cost = opt_->identityCost();
            // std::cout << "belief" << std::endl;
            auto *nbelief = new Belief();
            nbelief->motion = motion;
            nbelief->sigma_ = 2.0*Eigen::MatrixXd::Identity(2,2);
            nbelief->lambda_ = 0.0*Eigen::MatrixXd::Identity(2,2);
            nbelief->cost = motion->cost;
            nbelief->x = motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            nbelief->y = motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1];

            // std::cout << "belief" << std::endl;
            motion->beliefs.push_back(nbelief);
            nn_->add(motion);
            startMotions_.push_back(motion);
        }

        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    // No else
    // std::cout << "trying to solve" << std::endl;
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    // std::cout << "trying to solve" << std::endl;
    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();


    std::priority_queue< Belief* > BeliefQueue;

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> nbh;
    std::vector<Motion *> nbh_queue;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    std::vector<int> valid;
    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    Belief* newBelief;

    double stepsize = siC_->getPropagationStepSize();

    if (bestGoalMotion_)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_.value());

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    while (ptc == false)
    {
        iterations_++;
        // std::cout << "iteration " << iterations_ << std::endl;
        // if (iterations_ > 3){
        //     exit(0);
        // }
        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
            goal_s->sampleGoal(rstate);
        // else if (rng_.uniform01() < goalBias_){
        //     sampleUniform(rstate);
        //     rstate->as<RealVectorStateSpace::StateType>()->values[0] = 45.0;
        //     rstate->as<RealVectorStateSpace::StateType>()->values[1] = 85.0;
        // }
        else
        {
            // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
            // loop and return to try again
            if (!sampleUniform(rstate))
                continue;
        }

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        // std::cout << nmotion->beliefs[0]->x << " " << nmotion->beliefs[0]->y << std::endl;
        // std::exit(0);
        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);

        // std::cout << d << " " << maxDistance_ << std::endl;
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // std::cout << "NEW MOTION CANDIDATE " << dstate->as<ob::RealVectorStateSpace::StateType>()->values[0] << " " << dstate->as<ob::RealVectorStateSpace::StateType>()->values[1] << std::endl;
        // Check if the motion between the nearest state and the state to add is valid
        if (checkMotion(nmotion, dstate))
        {
            // create a motion
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
            
            // std::cout << " NEW MOTION " << dstate->as<ob::RealVectorStateSpace::StateType>()->values[0] << " " << dstate->as<ob::RealVectorStateSpace::StateType>()->values[1] << " " << nmotion->cost << std::endl;
            // auto *belief = new Belief();
            // belief->cost = motion->cost;
            // belief->x = motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            // belief->y = motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1];
            // belief->incCost = motion->incCost;
            // belief->motion = motion;
            // motion->beliefs.push_back(belief);

            // Find nearby neighbors of the new motion
            getNeighbors(motion, nbh);

            ++statesGenerated;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }
        
            if (useNewStateRejection_)
            {
                if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);
                    for (auto it = 0; it < nbh.size(); ++it){
                    for (auto i = nbh[it]->beliefs.begin(); i != nbh[it]->beliefs.end(); ++i)
                    BeliefQueue.emplace(*i);
                    }
                }
                else  // If the new motion does not improve the best cost it is ignored.
                {
                    si_->freeState(motion->state);
                    if (motion->control_)
                        siC_->freeControl(motion->control_);
                    delete motion;
                    continue;
                }
            }
            else
            {
                // add motion to the tree
                nn_->add(motion);
                motion->parent->children.push_back(motion);
                for (auto it = 0; it < nbh.size(); ++it){
                for (auto i = nbh[it]->beliefs.begin(); i != nbh[it]->beliefs.end(); ++i)
                BeliefQueue.emplace(*i);
                }
            }
            double distanceFromGoal;
            bool checkForSolution = false;
            while (BeliefQueue.size() > 0){

                // if (BeliefQueue.size() > 10){
                //     exit(0);
                // }
                
                // std::cout << "Belief queue size " << BeliefQueue.size() << std::endl;
                // std::cout << "pop "<< std::endl;
                Belief *belief = BeliefQueue.top();
                // std::cout << "pop "<< std::endl;
                BeliefQueue.pop();
                
                if (belief->deleted){
                    continue;
                }
                if (belief->parent == nullptr && belief->motion->parent != NULL) {  
                   continue;
                }
                // std::cout << "pop "<< std::endl;
                getNeighbors(belief->motion, nbh_queue); //returns a bunch of motions
                // std::cout << nbh_queue.size() << std::endl;
                // std::cout << "pop "<< std::endl;
                for (std::size_t i=0; i < nbh_queue.size(); ++i)
                {

                    // std::cout << "huh" << std::endl; 
                    // if (nbh_queue[i]->deleted){
                    //     continue;
                    // }

                    if (nbh_queue[i]->state != belief->motion->state) //if neighbor motion of belief is not the same motion that belief belongs to
                    {  
                        auto *motion = new Motion(siC_);
                        si_->copyState(motion->state, belief->motion->state);
                        double duration = 0.0;
                        Control *rctrl = motion->control_;
                        const ompl::base::RealVectorBounds &bounds = siC_->getControlSpace()->as<ompl::control::RealVectorControlSpace>()->getBounds();
                        const double diff_x = abs(nbh_queue[i]->state->as<ob::RealVectorStateSpace::StateType>()->values[0] - belief->motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                        const double diff_y = abs(nbh_queue[i]->state->as<ob::RealVectorStateSpace::StateType>()->values[1] - belief->motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1]);

                        unsigned int cd = ceil(std::max(diff_x, diff_y)/(bounds.high[0]*0.1));
                        rctrl->as<RealVectorControlSpace::ControlType>()->values[0] = diff_x/cd;
                        rctrl->as<RealVectorControlSpace::ControlType>()->values[1] = diff_y/cd;

                        Belief *dbelief = new Belief();

                        dbelief->parent = belief;
                        dbelief->motion = nbh_queue[i];
                        // std::cout << "here" << std::endl;

                        double cost = 0;

                        // unsigned int propCd = mypropagateWhileValid(belief, rctrl, cd, dbelief); //rstate should be new motion
                        unsigned int propCd = mypropagateAndCostWhileValid(belief, rctrl, cd, dbelief, cost); //rstate should be new motion
                        // std::cout << "here" << std::endl;
                        if (propCd == cd){
                            dbelief->x = dbelief->motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0];
                            dbelief->y = dbelief->motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1];
                            // dbelief->incCost = opt_->motionCost(dbelief->motion->state, belief->motion->state); //update inccost of new belief
                            dbelief->incCost = Cost(cost);
                            dbelief->cost = opt_->combineCosts(belief->cost, dbelief->incCost); //update cost of new belief
                            // if (belief->cost.value() > 250){
                            //     std::cout << belief->motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0] << " " << belief->motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1] << std::endl;
                            //     std::cout << dbelief->incCost.value() << " " << dbelief->cost.value() << std::endl;
                            // }
                            // std::cout << "here" << std::endl;

                            if (myisValid(dbelief))
                            {
                                if (appendBelief(dbelief->motion, dbelief)){
                                    //TODO: fix this because the state is not R2BeliefSpace but RealVectorStateSpace - define goalregion in this script.
                                    if (goal->isSatisfied(dbelief->motion->state, &distanceFromGoal))
                                    {
                                        motion->inGoal = true;
                                        goalMotions_.push_back(motion);
                                        checkForSolution = true;
                                    }
                                    // if (dbelief->x  > 40 && dbelief->y  > 50){
                                    // if ((dbelief->x - ){
                                        // std::cout << dbelief->x << " " << dbelief->y << " " << (dbelief->lambda_ + dbelief->sigma_).trace() << std::endl;
                                        // std::cout << "might find solution" << std::endl;
                                    // }

                                    // if (dbelief->y - )
                                    // std::cout << "here: " << belief->deleted << std::endl;
                                    // if (opt_->isCostBetterThan(dbelief->cost, dbelief->motion->cost){
                                    //     dbelief->motion->incCost = dbelief->incCost;
                                    //     dbelief->motion->parent = belief->motion;
                                    // }
                                    belief->children.push_back(dbelief);
                                    // std::cout << "after appending " << belief->motion->beliefs.size() << std::endl;
                                    // std::cout << "appending" << std::endl;
                                    BeliefQueue.emplace(dbelief);
                                }
                                else{
                                    // std::cout << "not appending" << std::endl;
                                    delete dbelief;
                                    // std::cout << "deleted" << std::endl;
                                }
                            }
                            else{
                                delete dbelief;
                            }
                            
                        }
                        else{
                            // std::cout <<"trying to delete" << std::endl;
                            // if (!dbelief->deleted)
                                delete dbelief;
                        }
                        if (motion->state)
                            si_->freeState(motion->state);
                        if (motion->control_)
                            siC_->freeControl(motion->control_);
                        delete motion;
                    }
                }
                // std::cout << "done!" << std::endl;
            
            }

            // bool checkForSolution = false;
            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            // double distanceFromGoal;
            // if (goal->isSatisfied(motion->state, &distanceFromGoal))
            // {
            //     motion->inGoal = true;
            //     goalMotions_.push_back(motion);
            //     checkForSolution = true;
            // }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                if (!bestGoalMotion_ && !goalMotions_.empty())
                {
                    // We have found our first solution, store it as the best. We only add one
                    // vertex at a time, so there can only be one goal vertex at this moment.
                    bestGoalMotion_ = goalMotions_.front();
                    for (auto &belief : bestGoalMotion_->beliefs)
                    {
                        if (belief->inGoal && opt_->isCostBetterThan(belief->cost, bestCost_))
                        {
                            bestCost_ = belief->cost;
                            updatedSolution = true;
                        }
                    }
                    // bestCost_ = bestGoalMotion_->cost;
                    // updatedSolution = true;
                    // double bestCost = 10000;
                    // Belief* result;
                    // for (auto ii:bestGoalMotion_->beliefs){
                    //     if (ii->cost.value() < bestCost){
                    //         bestCost = ii->cost.value();
                    //         result = ii;
                    //     }
                    // }

                    // while (result->parent != nullptr){
                    //     OMPL_INFORM("%f %f", result->x, result->y);
                    //     result = result->parent;
                    // }
                    // OMPL_INFORM("%f %f", result->x, result->y);
                    // OMPL_INFORM("Belief cost is : %f", bestCost );
                    
                    OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                "vertices in the graph)",
                                getName().c_str(), bestCost_.value(), iterations_, nn_->size());
                }
                else
                {
                    //TODO:2024: Need to check if the belief cost is the correct one...
                    // We already have a solution, iterate through the list of goal vertices
                    // and see if there's any improvement.
                    for (auto &goalMotion : goalMotions_)
                    {
                        for (auto &belief : goalMotion->beliefs)
                        {
                            // Is this goal motion better than the (current) best?
                            if (belief->inGoal && opt_->isCostBetterThan(belief->cost, bestCost_))
                            {
                                bestGoalMotion_ = goalMotion;
                                // bestCost_ = bestGoalMotion_->cost;
                                bestCost_ = belief->cost;
                                updatedSolution = true;

                                // Check if it satisfies the optimization objective, if it does, break the for loop
                                if (opt_->isSatisfied(bestCost_))
                                {
                                    break;
                                }
                            }
                        }
                    }
                }

                if (updatedSolution)
                {

                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution =
                            bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approxDist)
            {
                approxGoalMotion = motion;
                approxDist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
            break;

        
    }

    // Add our solution (if it exists)
    Motion *newSolution = nullptr;
    if (bestGoalMotion_)
    {
        // We have an exact solution
        newSolution = bestGoalMotion_;
    }
    else if (approxGoalMotion)
    {
        // We don't have a solution, but we do have an approximate solution
        newSolution = approxGoalMotion;
    }
    // No else, we have nothing

    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        // set the solution path
        auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
            psol.setApproximate(approxDist);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    // No else, we have nothing

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false

    // std::cout << "done1" << std::endl;
    return {newSolution != nullptr, bestGoalMotion_ == nullptr};
}

void ompl::control::RRBT::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        // nn_->nearestK(motion, k, nbh);
        int maximumi= 20;
        nn_->nearestK(motion, std::min(maximumi, int(std::ceil(k_rrt_ * log(cardDbl)))) , nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}

void ompl::control::RRBT::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::control::RRBT::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::control::RRBT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            // if (motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0] > 40 && motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1] > 50)
            // OMPL_INFORM("%f %f", motion->state->as<ob::RealVectorStateSpace::StateType>()->values[0], motion->state->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            if (motion->deleted)
                continue;
            // OMPL_INFORM("Deleting motion");
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control_)
                siC_->freeControl(motion->control_);
            // OMPL_INFORM("Size is %d", motion->beliefs.size());
            // for (int i = 0; i < motion->beliefs.size(); ++i){
            //     if (motion->beliefs[i]->deleted != true && motion->beliefs[i]->motion != nullptr)
            //     delete motion->beliefs[i];
            // }
            motion->beliefs.clear();
            // OMPL_INFORM("Really deleting motion");
            delete motion;
        }
    }
}

bool ompl::control::RRBT::checkMotion(Motion * nmotion, State* dstate)
{
    double x = dstate->as<ob::RealVectorStateSpace::StateType>()->values[0];
    double y = dstate->as<ob::RealVectorStateSpace::StateType>()->values[1];
    if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0){
        return false;
    }

    const ompl::base::RealVectorBounds &bounds = siC_->getControlSpace()->as<ompl::control::RealVectorControlSpace>()->getBounds();
    const double diff_x = abs(dstate->as<ob::RealVectorStateSpace::StateType>()->values[0] - nmotion->state->as<ob::RealVectorStateSpace::StateType>()->values[0]);
    const double diff_y = abs(dstate->as<ob::RealVectorStateSpace::StateType>()->values[1] - nmotion->state->as<ob::RealVectorStateSpace::StateType>()->values[1]);

    unsigned int cd = ceil(std::max(diff_x, diff_y)/(bounds.high[0]*0.1));
    auto *motion = new Motion(siC_);
    siC_->nullControl(motion->control_);
    Control *rctrl = motion->control_;
    // cd = 10*cd;
    // std::cout << cd << std::endl;
    rctrl->as<RealVectorControlSpace::ControlType>()->values[0] = diff_x/cd;
    rctrl->as<RealVectorControlSpace::ControlType>()->values[1]= diff_y/cd;

    auto *result = new Belief();
    // std::cout << "number of beliefs for motion" << nmotion->beliefs.size() << std::endl;
    // if (nmotion->beliefs.size() == 0){
    //     std::cout << "SHIT" << std::endl;
    //     exit(0);
    // }
    // if (nmotion->beliefs.size() > 10){
    //     for (auto it:nmotion->beliefs){
    //         std::cout << it->x << " " << it->y << " " << it->cost << " " << (it->sigma_ + it->lambda_).trace() << std::endl;
    //     }
    //     exit(0);
    // }
    for (auto it:nmotion->beliefs){
        // std::cout << "propagating from: "<< it->x << " " << it->y << " to " << dstate->as<ob::RealVectorStateSpace::StateType>()->values[0] << " " << dstate->as<ob::RealVectorStateSpace::StateType>()->values[1] << std::endl;
        unsigned int propCd = mypropagateWhileValid(it, rctrl, cd, result);
        result->x  = dstate->as<ob::RealVectorStateSpace::StateType>()->values[0];
        result->y = dstate->as<ob::RealVectorStateSpace::StateType>()->values[1];
        // propCd = cd;
        // std::cout << "motion is " << propCd << " " << cd << std::endl;
        if (propCd == cd && myisValid(result)){
            return true;
        }
    }

    delete result;
    si_->freeState(motion->state);
    siC_->freeControl(motion->control_);
    delete motion;

    return false;
}

unsigned int ompl::control::RRBT::mypropagateWhileValid(const Belief* belief, const Control *control,
                                                                  int steps, Belief* result) const
{
    if (steps == 0)
    {
        // result->parent = belief->parent;
        result->cost = belief->cost;
        result->incCost = belief->incCost;
        result->sigma_ = belief->sigma_;
        result->lambda_ = belief->lambda_;
        return 0;
    }

    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    // perform the first step of propagation
    mypropagate(belief, control, signedStepSize, result);

    // if we found a valid state after one step, we can go on
    if (myisValid(result))
    {
        Belief *temp1 = result;
        Belief *temp2 = new Belief();
        Belief *toDelete = temp2;
        unsigned int r = steps;

        // for the remaining number of steps
        for (int i = 1; i < steps; ++i)
        {
            mypropagate(temp1, control, signedStepSize, temp2);
            if (myisValid(temp2))
                std::swap(temp1, temp2);
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }

        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure result contains that information
        if (result->x != temp1->x && result->y != temp1->y){
            result = temp1;
        }
        // free the temporary memory
        delete toDelete;

        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    if (result->x != belief->x && result->y != belief->y){
        result->cost = belief->cost;
        result->incCost = belief->incCost;
        result->sigma_ = belief->sigma_;
        result->lambda_ = belief->lambda_;
    }
    return 0;
}

unsigned int ompl::control::RRBT::mypropagateAndCostWhileValid(const Belief* belief, const Control *control,
                                                                  int steps, Belief* result, double &cost) const
{

    cost = 0;
    if (steps == 0)
    {
        // result->parent = belief->parent;
        result->cost = belief->cost;
        result->incCost = belief->incCost;
        result->sigma_ = belief->sigma_;
        result->lambda_ = belief->lambda_;
        return 0;
    }

    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    // perform the first step of propagation
    mypropagate(belief, control, signedStepSize, result);

    cost += opt_->motionCost(belief->motion->state, result->motion->state).value(); 

    // if we found a valid state after one step, we can go on
    if (myisValid(result))
    {
        Belief *temp1 = result;
        Belief *temp2 = new Belief();
        Belief *toDelete = temp2;
        unsigned int r = steps;

        // for the remaining number of steps
        for (int i = 1; i < steps; ++i)
        {
            mypropagate(temp1, control, signedStepSize, temp2);
            if (myisValid(temp2))
            {
                cost += opt_->motionCost(temp1->motion->state, temp2->motion->state).value();
                std::swap(temp1, temp2);
            }
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }

        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure result contains that information
        if (result->x != temp1->x && result->y != temp1->y){
            result = temp1;
        }
        // free the temporary memory
        delete toDelete;

        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    if (result->x != belief->x && result->y != belief->y){
        result->cost = belief->cost;
        result->incCost = belief->incCost;
        result->sigma_ = belief->sigma_;
        result->lambda_ = belief->lambda_;
    }
    return 0;
}

void ompl::control::RRBT::mypropagate(const Belief *belief, const control::Control* control, const double duration, Belief *result) const
{
    //=========================================================================
    // Get CX vector (RRT near vertex)
    //=========================================================================

    double x_pose = belief->x;
    double y_pose = belief->y;
    //=========================================================================
    // Get CX vector (RRT random vertex)
    //=========================================================================

    double x_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    double y_pose_reference = control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
    //=========================================================================
    // Compute control inputs (dot(dot(x)) dot(dot(y)) dot(dot(z))) with PD controller
    //=========================================================================
    double u_0 = B_ol_(0, 0) * (x_pose_reference - x_pose); //double u_0 = B_cl_d_(0, 0) * (x_pose_reference - x_pose);
    double u_1 = B_ol_(0, 0) * (y_pose_reference - y_pose); //double u_1 = B_cl_d_(1, 1) * (y_pose_reference - y_pose);
    //=========================================================================
    // Propagate mean
    //=========================================================================
    result->x = x_pose + duration * x_pose_reference;
    result->y = y_pose + duration * y_pose_reference;

    //=========================================================================
    // Propagate covariance in the equivalent closed loop system
    //=========================================================================

    Eigen::Matrix2d sigma_from = belief->sigma_;
    Eigen::Matrix2d lambda_from = belief->lambda_;
    Eigen::Matrix2d sigma_pred = F*sigma_from*F + Q;

    Mat lambda_pred, K;

    //scenario 1
    // if (1 == 2){
    // if (x_pose + duration * u_0 > 75 && y_pose + duration * u_1 < 30){ //scenario 2
    // std::cout << "reached measurement region" << std::endl;
    // if (true){ //scenario 3
    if (x_pose + duration * u_0 > 0.0 && x_pose + duration * u_0 < 55 && y_pose + duration * u_1 < 25){  //scenario 4
        Mat R = 0.1*Eigen::MatrixXd::Identity(2, 2);
        // std::cout << "what1" << std::endl;
        Mat S = (H * sigma_pred * H.transpose()) + R;
        // std::cout << "what2" << std::endl;
        K = (sigma_pred * H.transpose()) * S.inverse();
        // std::cout << "what3" << std::endl;
        lambda_pred = A_cl_*lambda_from*A_cl_;
        // std::cout << "what4" << std::endl;
    }
    else{
        K = Eigen::MatrixXd::Zero(2, 2);
        lambda_pred = lambda_from;
    }
    Mat sigma_to = (I - (K*H)) * sigma_pred;
    Mat lambda_to = lambda_pred + K*H*sigma_pred;
    result->sigma_ = sigma_to;
    result->lambda_ = lambda_to;
}

bool ompl::control::RRBT::inCollision(const Belief *belief, 
                         double X1, double Y1, 
                         double X2, double Y2) const
{ 
    

        Eigen::Matrix2d Cov = belief->sigma_ + belief->lambda_;
        double R = 2*sqrt(Cov(0,0));

        double Xc = belief->x;
        double Yc = belief->y;


        // Find the nearest point on the  
        // rectangle to the center of  
        // the circle 
        double Xn = std::max(X1, std::min(Xc, X2)); 
        double Yn = std::max(Y1, std::min(Yc, Y2)); 
        
        // Find the distance between the  
        // nearest point and the center  
        // of the circle 
        // Distance between 2 points,  
        // (x1, y1) & (x2, y2) in  
        // 2D Euclidean space is 
        // ((x1-x2)**2 + (y1-y2)**2)**0.5 
        double Dx = Xn - Xc; 
        double Dy = Yn - Yc; 
        return (Dx * Dx + Dy * Dy) <= R * R; 
}

bool ompl::control::RRBT::inCollision(const Belief *state) const{

    //scenario 1
    // if (inCollision(state, 0.0, 60, 40, 80)){
    //     return true;
    // }
    // else if (inCollision(state, 50, 60, 100, 80)){
    //     return true;
    // }
    // return false;

    //scenario 2
    if (inCollision(state, 0.0, 50, 40, 80)){
        return true;
    }
    else if (inCollision(state, 50, 50, 100, 80)){
        return true;
    }
    return false;
}

bool ompl::control::RRBT::myisValid(const Belief *state) const
{
    // for each obstacles
    // check if mean + 2sigma is in collision
    // return validity
    // return true;
    double x = state->x;
    double y = state->y;
    if (x > 100.0 || x < 0.0 || y < 0.0 || y > 100.0){
        return false;
    }

    //=========================================================================
	// Extract the component of the state and cast it to what we expect
	//=========================================================================
	double z_pose;
	Eigen::MatrixXf PX(3, 3); PX.setZero();

	z_pose = 4.0;
    PX(0,0) = state->lambda_(0,0) + state->sigma_(0,0);
    PX(1,1) = state->lambda_(1,1) + state->sigma_(1,1);
    PX(2,2) = 0.1;
	// PX(0,0) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[0];
	// PX(1,1) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[1];
	// PX(2,2) = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(4)->values[2];


	//=========================================================================
	// Probabilistic collision checker
	//=========================================================================
	bool valid = false;
	for (int o = 0; o < n_obstacles_; o++) {
		if (not HyperplaneCCValidityChecker(A_list_.at(o), B_list_.at(o), x, y, z_pose, PX)) {
			goto exit_switch;
		}
	}
	valid = true;

	exit_switch:;
	return valid;

    // return true;
    return !(inCollision(state));
}

bool ompl::control::RRBT::HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const {
	
    
    bool valid = false;
	double PV, b_bar;
	Eigen::MatrixXf Pv_2;

	for (int i = 0; i < 6; i++) {
		Pv_2 = A.row(i) * PX * A.row(i).transpose();
		PV = sqrt(Pv_2(0, 0));

		b_bar = sqrt(2) * PV * erf_inv_result_;

		if(x_pose * A(i, 0) + y_pose * A(i, 1) + z_pose * A(i, 2) >= (B(i, 0) + b_bar)) {
			valid = true;
			break;
		}
	}
    // if (valid){
    //     if (x_pose < 40 && y_pose > 50 && y_pose < 80){
    //         std::cout << x_pose << " " << y_pose << std::endl;
    //     } 
    //     // if (x_pose > 50 && y_pose > 50 && y_pose < 80){
    //     //     std::cout << x_pose << " " << y_pose << std::endl;
    //     // } 
    // }

    // if (valid == false){
    //     std::cout << "false!!" << std::endl;
    //     std::cout << x_pose << " " << y_pose << std::endl;
    // }
	return valid;
}

bool ompl::control::RRBT::appendBelief(Motion* motion, Belief* newbelief)
{

    std::vector<Belief *> nbeliefs;
    // std::cout << "size of motion beliefs" << std::endl;
    // std::cout << motion->beliefs.size() << std::endl;
    // std::cout << "size of motion beliefs " << motion->beliefs.size() << std::endl;
    // if (newbelief->lambda_.trace() > 0){
        // std::cout << newbelief->x << " " << newbelief->y << std::endl;
    // }
    for (auto & existingbelief : motion->beliefs)
    {
        if (existingbelief->parent != nullptr && existingbelief->deleted != true){
        // if (!existingbelief->deleted)
            if (epsilon_dominates(existingbelief, newbelief)) return false; //if new belief is dominated by any existing belief, return false.
        }
        // else{
            // std::cout << "existing belief is to be deleted" << std::endl;
        // }
    }
    
    // double min_cost = newbelief->cost.value(); //very high value, should change
    motion->cost = ob::Cost(10000);
    // for (auto & existingbelief : motion->beliefs)
    // {
    //     std::cout << existingbelief->x << " " << existingbelief->y << " " << (existingbelief->lambda_ + existingbelief->sigma_).trace() << " " << existingbelief->cost << std::endl;
    // }

    // std::cout << "Pruning dominated beliefs" << std::endl;
    for (auto & existingbelief : motion->beliefs)
    {
        // std::cout << newbelief->x << " " << newbelief->y << " " << (newbelief->lambda_).trace() << " " << newbelief->sigma_.trace() << " " << newbelief->cost.value() << std::endl;
        // std::cout << (existingbelief->lambda_).trace() << " " << existingbelief->sigma_.trace() << " " << existingbelief->cost.value() << std::endl;
        
        // std::cout << "new existing belief! "<< std::endl;
        if (existingbelief->parent == nullptr && existingbelief->deleted){
            // std::cout << "trying to delete existing belief" << std::endl;
            // delete existingbelief;
            toBeDeleted.push_back(existingbelief);
        }
        else if (!dominates(newbelief, existingbelief)){
            nbeliefs.push_back(existingbelief);
            if (motion->cost.value() > existingbelief->cost.value()){
                motion->cost = existingbelief->cost;
                // std::cout << "changing parent" << std::endl;
                // motion->parent = existingbelief->parent->motion;
                // std::cout << "changed parent" << std::endl;
            }
        }
        else{
            // if (!existingbelief->deleted){
            // std::cout << "trying to delete existing belief 2" << std::endl;
            // delete existingbelief;
            // std::cout << "deleted existingbelief" << std::endl;
            // }
            // delete existingbelief;
            toBeDeleted.push_back(existingbelief);
        }
    }
    // std::cout << "after deletion" << std::endl;
    motion->beliefs.clear();
    // std::cout << newbelief->x << " " << newbelief->y << " " << (newbelief->sigma_ + newbelief->lambda_).trace() << std::endl;
    nbeliefs.push_back(newbelief);
    if (motion->cost.value() > newbelief->cost.value()){
                motion->cost = newbelief->cost;
                motion->parent = newbelief->parent->motion;
    }
    motion->beliefs = nbeliefs;
    // std::cout << newbelief->x << " " << newbelief->y << " motion cost is " << motion->cost.value() << "new belief cost is: " << newbelief->cost.value() << std::endl;
    // std::cout << motion->beliefs.size() << std::endl;
    return true;
}

void ompl::control::RRBT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

int ompl::control::RRBT::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;


    // Variable
    // The queue of Motions to process:
    std::queue<Motion *, std::deque<Motion *>> motionQueue;
    // The list of leaves to prune
    std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
    // The list of chain vertices to recheck after pruning
    std::list<Motion *> chainsToRecheck;

    // Clear the NN structure:
    nn_->clear();

    // Put all the starts into the NN structure and their children into the queue:
    // We do this so that start states are never pruned.
    for (auto &startMotion : startMotions_)
    {
        // Add to the NN
        nn_->add(startMotion);

        // Add their children to the queue:
        addChildrenToList(&motionQueue, startMotion);
    }

    while (motionQueue.empty() == false)
    {
        // Test, can the current motion ever provide a better solution?
        if (keepCondition(motionQueue.front(), pruneTreeCost))
        {
            // Yes it can, so it definitely won't be pruned
            // Add it back into the NN structure
            nn_->add(motionQueue.front());

            // Add it's children to the queue
            addChildrenToList(&motionQueue, motionQueue.front());
        }
        else
        {
            // No it can't, but does it have children?
            if (motionQueue.front()->children.empty() == false)
            {
                // Yes it does.
                // We can minimize the number of intermediate chain motions if we check their children
                // If any of them won't be pruned, then this motion won't either. This intuitively seems
                // like a nice balance between following the descendents forever.

                // Variable
                // Whether the children are definitely to be kept.
                bool keepAChild = false;

                // Find if any child is definitely not being pruned.
                for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                {
                    // Test if the child can ever provide a better solution
                    keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                }

                // Are we *definitely* keeping any of the children?
                if (keepAChild)
                {
                    // Yes, we are, so we are not pruning this motion
                    // Add it back into the NN structure.
                    nn_->add(motionQueue.front());
                }
                else
                {
                    // No, we aren't. This doesn't mean we won't though
                    // Move this Motion to the temporary list
                    chainsToRecheck.push_back(motionQueue.front());
                }

                // Either way. add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No, so we will be pruning this motion:
                leavesToPrune.push(motionQueue.front());
            }
        }

        // Pop the iterator, std::list::erase returns the next iterator
        motionQueue.pop();
    }

    // We now have a list of Motions to definitely remove, and a list of Motions to recheck
    // Iteratively check the two lists until there is nothing to to remove
    while (leavesToPrune.empty() == false)
    {
        // First empty the current leaves-to-prune
        while (leavesToPrune.empty() == false)
        {
            // If this leaf is a goal, remove it from the goal set
            if (leavesToPrune.front()->inGoal == true)
            {
                // Warn if pruning the _best_ goal
                if (leavesToPrune.front() == bestGoalMotion_)
                {
                    OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                }
                // Remove it
                goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                    goalMotions_.end());
            }

            // Remove the leaf from its parent
            removeFromParent(leavesToPrune.front());

            // Erase the actual motion
            // First free the state
            si_->freeState(leavesToPrune.front()->state);

            // then delete the pointer
            delete leavesToPrune.front();

            // And finally remove it from the list, erase returns the next iterator
            leavesToPrune.pop();

            // Update our counter
            ++numPruned;
        }

        // Now, we need to go through the list of chain vertices and see if any are now leaves
        auto mIter = chainsToRecheck.begin();
        while (mIter != chainsToRecheck.end())
        {
            // Is the Motion a leaf?
            if ((*mIter)->children.empty() == true)
            {
                // It is, add to the removal queue
                leavesToPrune.push(*mIter);

                // Remove from this queue, getting the next
                mIter = chainsToRecheck.erase(mIter);
            }
            else
            {
                // Is isn't, skip to the next
                ++mIter;
            }
        }
    }

    // Now finally add back any vertices left in chainsToReheck.
    // These are chain vertices that have descendents that we want to keep
    for (const auto &r : chainsToRecheck)
        // Add the motion back to the NN struct:
        nn_->add(r);

    // All done pruning.
    // Update the cost at which we've pruned:
    prunedCost_ = pruneTreeCost;

    // And if we're using the pruned measure, the measure to which we've pruned
    if (usePrunedMeasure_)
    {
        prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }

    return numPruned;
}

void ompl::control::RRBT::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::control::RRBT::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::control::RRBT::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::control::RRBT::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::control::RRBT::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::control::RRBT::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::control::RRBT::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::control::RRBT::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::control::RRBT::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}

bool ompl::control::RRBT::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::control::RRBT::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}

bool ompl::control::RRBT::epsilon_dominates(Belief *a, Belief *b) const{
        //returns true if vertex a epsilon dominates vertex b
        double eps = 2.0;
        // std::cout << a->x << " "  << a->sigma_.trace() << " " << a->lambda_.trace() << " " << a->cost << std::endl;
        // std::cout << b->sigma_.trace() << " " << b->lambda_.trace() << " " << b->cost << std::endl;
        // std::cout << "cost is " << a->cost << " " << b->cost << std::endl;
        if (a->sigma_.trace() >= b->sigma_.trace()  + eps){
            return false;
        }
        else if (a->lambda_.trace() >= b->lambda_.trace() + eps){
            return false;
        }
        // else return (opt_->isCostBetterThan(a->cost, b->cost));  //TODO: add epsilon
        else return (opt_->isCostBetterThan(a->cost, ob::Cost(b->cost.value() + 0.01)));  //TODO: add epsilon
        return true;
    }

bool ompl::control::RRBT::dominates(Belief *a, Belief*b) const{
    //returns true if vertex a dominates vertex b

    if (a->sigma_.trace() >= b->sigma_.trace())
        return false;
    else if (a->lambda_.trace() >= b->lambda_.trace())
        return false;
    else if (opt_->isCostBetterThan(b->cost, a->cost)){
        return false;
    };
    return true;
}


// double ompl::control::RRBT::chanceConstrainedGoalRegion(Belief *a, Belief *b) const{


// }