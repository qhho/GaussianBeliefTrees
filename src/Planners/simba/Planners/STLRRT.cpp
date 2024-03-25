/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Matt Maly */

#include "STLRRT.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "../stl/STLProductGraph.h"
#include "../stl/STLProblemDefinition.hpp"
#include "ompl/datastructures/PDF.h"
#include "ompl/util/Console.h"
#include <algorithm>
#include <unordered_map>
#include <limits>
#include <map>
#include <utility>
#include <vector>
#include <fstream>

#include <cstdio>
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include  <random>
#include  <iterator>

template<typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));
    return start;
}

template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
}

ompl::control::STLRRT::STLRRT(const STLSpaceInformationPtr &stlsi, ProductGraphPtr a, double exploreTime)
  : ompl::base::Planner(stlsi, "STLRRT")
  , stlsi_(stlsi.get())
  , abstraction_(std::move(a))
  , exploreTime_(exploreTime)
{
    specs_.approximateSolutions = true;

    Planner::declareParam<double>("goal_bias", this, &STLRRT::setGoalBias, &STLRRT::getGoalBias, "0.:.05:1.");
    // Planner::declareParam<double>("traj_bias", this, &STLRRT::setTrajBias, &STLRRT::setTrajBias, "0.:.05:1.");
}

ompl::control::STLRRT::~STLRRT()
{
    clearMotions();
}

void ompl::control::STLRRT::setup()
{
    base::Planner::setup();
}

void ompl::control::STLRRT::clear()
{
    base::Planner::clear();
    availDist_.clear();
    abstractInfo_.clear();
    clearMotions();
}

ompl::base::PlannerStatus ompl::control::STLRRT::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // \todo make solve work when called more than once!
    checkValidity();
    const base::State *start = pis_.nextStart();
    prodStart_ = stlsi_->getProdGraphState(start);

    if (pis_.haveMoreStartStates())
        OMPL_WARN("Multiple start states given. Using only the first start state.");

    auto *startMotion = new Motion(stlsi_);
    si_->copyState(startMotion->state, start);
    stlsi_->nullControl(startMotion->control);
    startMotion->abstractState = prodStart_;
    motions_.push_back(startMotion);
    abstractInfo_[prodStart_].addMotion(startMotion);
    if (!abstractInfo_[prodStart_].nn)
    {
        abstractInfo_[prodStart_].nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        abstractInfo_[prodStart_].nn->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunction(a, b); });
    }
    abstractInfo_[prodStart_].nn->add(startMotion);
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        nn_->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunction(a, b); });
    }
    nn_->add(startMotion);
    updateWeight(prodStart_);
    availDist_.add(prodStart_, abstractInfo_[prodStart_].weight);

    abstraction_->buildGraph(prodStart_, [this](ProductGraph::State *as)
                             {
                                 initAbstractInfo(as);
                             });

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = stlsi_->allocControlSampler();

    bool solved = false;
    Motion *soln;
    int i = 0;
    while (ptc() == false && !solved)
    {
        ++i;
        const std::vector<ProductGraph::State *> lead =
            abstraction_->computeLead(prodStart_, [this](ProductGraph::State *a, ProductGraph::State *b)
                                      {
                                          return abstractEdgeWeight(a, b);
                                      });
        for (auto l : lead){
            std::cout << l->getCosafeState() << " " << l->getDecompRegion() << ", " ;
            // std::vector<double > coord(2, 0.0);
            // abstraction_->getDecomp()->sampleFromRegion(l->getDecompRegion(), rng_, coord);
            // abstractInfo_[l].x = coord[0];
            // abstractInfo_[l].y = coord[1];
        }
        std::cout << std::endl;
        buildAvail(lead);
        solved = explore(lead, soln, exploreTime_);
    }

    // std::cout << i << std::endl;

    if (solved)
    {
        // build solution path
        std::vector<Motion *> path;
        while (soln != nullptr)
        {
            path.push_back(soln);
            soln = soln->parent;
        }
        auto pc(std::make_shared<PathControl>(si_));
        for (int i = path.size() - 1; i >= 0; --i)
        {
            if (path[i]->parent != nullptr)
                pc->append(path[i]->state, path[i]->control, path[i]->steps * stlsi_->getPropagationStepSize());
            else
                pc->append(path[i]->state);
        }
        pdef_->addSolutionPath(pc);
    }
    // std::fstream file1("medium_scene2_num.txt", std::fstream::out | std::fstream::app);
    // file1 << motions_.size() << std::endl;
    // file1.close();
    OMPL_INFORM("Created %u states", motions_.size());
    num_vertices_ = motions_.size();
    return {solved, false};
}

void ompl::control::STLRRT::getTree(std::vector<base::State *> &tree) const
{
    tree.resize(motions_.size());
    for (unsigned int i = 0; i < motions_.size(); ++i)
        tree[i] = motions_[i]->state;
}

std::vector<ompl::control::ProductGraph::State *>
ompl::control::STLRRT::getHighLevelPath(const std::vector<base::State *> &path, ProductGraph::State *start) const
{
    std::vector<ProductGraph::State *> hlPath(path.size());
    hlPath[0] = (start != nullptr ? start : stlsi_->getProdGraphState(path[0]));
    for (unsigned int i = 1; i < path.size(); ++i)
    {
        hlPath[i] = stlsi_->getProdGraphState(path[i]);
        if (!hlPath[i]->isValid())
            OMPL_WARN("High-level path fails automata");
    }
    return hlPath;
}

ompl::control::STLRRT::Motion::Motion(const SpaceInformation *si)
  : state(si->allocState()), control(si->allocControl())
{
}

ompl::control::STLRRT::Motion::~Motion() = default;


void ompl::control::STLRRT::ProductGraphStateInfo::addMotion(Motion *m)
{
    motionElems[m] = motions.add(m, 1.);
}

double ompl::control::STLRRT::updateWeight(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = abstractInfo_[as];
    /* \todo weight should include freeVolume, for cases in which decomposition
       does not respect obstacles. */
    info.weight = ((info.motions.size() + 1) * info.volume) / (info.autWeight * (info.numSel + 1) * (info.numSel + 1));
    return info.weight;
}

void ompl::control::STLRRT::initAbstractInfo(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = abstractInfo_[as];
    info.numSel = 0;
    info.pdfElem = nullptr;
    info.volume = abstraction_->getRegionVolume(as);
    unsigned int autDist = std::max(abstraction_->getCosafeAutDistance(as), abstraction_->getSafeAutDistance(as));
    std::vector<double > coord(2, 0.0);
    abstraction_->getDecomp()->sampleFromRegion(as->getDecompRegion(), rng_, coord);
    std::cout << coord[0] << " " << coord[1] << std::endl;
    abstractInfo_[as].x = coord[0];
    abstractInfo_[as].y = coord[1];
    //\todo try something larger than epsilon
    if (autDist == 0)
        info.autWeight = std::numeric_limits<double>::epsilon();
    else
        info.autWeight = autDist;
    info.weight = info.volume / info.autWeight;
    if (!info.nn)
    {
        info.nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        info.nn->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    }
}

void ompl::control::STLRRT::buildAvail(const std::vector<ProductGraph::State *> &lead)
{
    for (unsigned int i = 0; i < availDist_.size(); ++i)
        abstractInfo_[availDist_[i]].pdfElem = nullptr;
    availDist_.clear();
    unsigned int numTreePts = 1;
    bool empty = false;
    for (int i = lead.size() - 1; i >= 0; --i)
    {
        ProductGraph::State *as = lead[i];
        ProductGraphStateInfo &info = abstractInfo_[as];
        if (!info.motions.empty())
        {
        info.pdfElem = availDist_.add(as, info.weight);
        numTreePts += info.motions.size();
            // if (rng_.uniform01() < 0.5)
            //     break;
        }
    }
}

bool ompl::control::STLRRT::explore(const std::vector<ProductGraph::State *> &lead, Motion *&soln, double duration)
{
    bool solved = false;
    base::PlannerTerminationCondition ptc = base::timedPlannerTerminationCondition(duration);
    base::GoalPtr goal = pdef_->getGoal();

    // base::Goal *goal_is = pdef_->getGoal().get();
    // auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal_is);

    
    auto *rmotion = new Motion(stlsi_);
    base::State *rstate = rmotion->state;
    std::vector<double > coord(2, 0.0);
    // std::cout << "1" << std::endl;
    while (!ptc() && !solved)
    {
        // std::cout << "2" << std::endl;
        ProductGraph::State *as = availDist_.sample(rng_.uniform01());
        ++abstractInfo_[as].numSel;
        updateWeight(as);
        PDF<Motion *> &motions = abstractInfo_[as].motions;
        ProductGraph::State *ls = *select_randomly(lead.begin(), lead.end());
        // PDF<Motion *> &motions = abstractInfo_[as].motions;
        /* sample random state (with goal biasing) */
        // if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        //     goal_s->sampleGoal(rstate);
        
        Motion *v;
        if (rng_.uniform01() < 1 - trajBias_){
            sampler_->sampleUniform(rstate);
            double val = rng_.uniform01();
            rstate->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setSigma(2*val);
            v = nn_->nearest(rmotion);
            // v = abstractInfo_[as].nn->nearest(rmotion);
            Control *rctrl = stlsi_->allocControl();
            controlSampler_->sampleNext(rctrl, v->control, v->state);
            unsigned int cd =
                controlSampler_->sampleStepCount(stlsi_->getMinControlDuration(), stlsi_->getMaxControlDuration());

            base::State *newState = si_->allocState();
            cd = stlsi_->propagateWhileValid(v->state, rctrl, cd, newState);
            if (cd < stlsi_->getMinControlDuration())
            {
                si_->freeState(newState);
                stlsi_->freeControl(rctrl);
                continue;
            }
            //NEED TO ADD MULTIPLE STATES HERE INSTEAD...
            auto *m = new Motion();
            m->state = newState;
            m->control = rctrl;
            m->steps = cd;
            m->parent = v;
            // Since the state was determined to be valid by SpaceInformation, we don't need to check automaton states
            m->abstractState = stlsi_->getProdGraphState(m->state);
            motions_.push_back(m);
            nn_->add(m);
            updateWeight(m->abstractState);

            solved = goal->isSatisfied(m->state);
            if (solved)
            {
                soln = m;
                break;
            }
        }
        else{
            abstraction_->getDecomp()->sampleFromRegion(as->getDecompRegion(), rng_, coord);
            rstate->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setXY(coord[0], coord[1]);
            // rstate->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setXY(abstractInfo_[ls].x, abstractInfo_[ls].y);
            double val = rng_.uniform01();
            rstate->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setSigma(2*val);
            v = abstractInfo_[as].nn->nearest(rmotion);
            // v = nn_->nearest(rmotion);
            // if (!motions.empty()){
            // PDF<Motion *>::Element *velem = abstractInfo_[as].motionElems[v];
            // double vweight = motions.getWeight(velem);
            // if (vweight > 1e-20)
            //     motions.update(velem, vweight / (vweight + 1.));
            // }
            Control *rctrl = stlsi_->allocControl();
            controlSampler_->sampleNext(rctrl, v->control, v->state);
            unsigned int cd =
                controlSampler_->sampleStepCount(stlsi_->getMinControlDuration(), stlsi_->getMaxControlDuration());

            base::State *newState = si_->allocState();
            cd = stlsi_->propagateWhileValid(v->state, rctrl, cd, newState);
            if (cd < stlsi_->getMinControlDuration())
            {
                si_->freeState(newState);
                stlsi_->freeControl(rctrl);
                continue;
            }
            // std::cout << "here4" << std::endl;
            //NEED TO ADD MULTIPLE STATES HERE INSTEAD...
            auto *m = new Motion();
            m->state = newState;
            m->control = rctrl;
            m->steps = cd;
            m->parent = v;
            // Since the state was determined to be valid by SpaceInformation, we don't need to check automaton states
            m->abstractState = stlsi_->getProdGraphState(m->state);
            motions_.push_back(m);

            abstractInfo_[m->abstractState].addMotion(m);
            abstractInfo_[m->abstractState].nn->add(m);
            nn_->add(m);
            updateWeight(m->abstractState);

        // update weight if hl state already exists in avail
            if (abstractInfo_[m->abstractState].pdfElem != nullptr)
                availDist_.update(abstractInfo_[m->abstractState].pdfElem, abstractInfo_[m->abstractState].weight);
            else
            {
                // otherwise, only add hl state to avail if it already exists in lead
                if (std::find(lead.begin(), lead.end(), m->abstractState) != lead.end())
                {
                    PDF<ProductGraph::State *>::Element *elem =
                        availDist_.add(m->abstractState, abstractInfo_[m->abstractState].weight);
                    abstractInfo_[m->abstractState].pdfElem = elem;
                }
            }
            solved = goal->isSatisfied(m->state);
            if (solved)
            {
                soln = m;
                break;
            }
        }
    }

    if (rmotion->state)
            si_->freeState(rmotion->state);
        if (rmotion->control)
            stlsi_->freeControl(rmotion->control);
        delete rmotion;
    
    return solved;
}

double ompl::control::STLRRT::abstractEdgeWeight(ProductGraph::State *a, ProductGraph::State *b) const
{
    const ProductGraphStateInfo &infoA = abstractInfo_.find(a)->second;
    const ProductGraphStateInfo &infoB = abstractInfo_.find(b)->second;
    return 1. / (infoA.weight * infoB.weight);
}

void ompl::control::STLRRT::clearMotions()
{
    availDist_.clear();
    for (auto m : motions_)
    {
        if (m->state != nullptr)
            si_->freeState(m->state);
        if (m->control != nullptr)
            stlsi_->freeControl(m->control);
        delete m;
    }
    motions_.clear();
    pis_.clear();
    pis_.update();
}