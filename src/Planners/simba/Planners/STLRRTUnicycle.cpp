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

/* Author: Matt Maly, Qi Heng Ho */

#include "STLRRTUnicycle.h"
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


/*
 void interpolated()
 {
     std::vector<base::State *> newStates;
     const int segments = states_.size() - 1;
  
     for (int i = 0; i < segments; ++i)
     {
         base::State *s1 = states_[i];
         base::State *s2 = states_[i + 1];
  
         newStates.push_back(s1);
         unsigned int n = si_->getStateSpace()->validSegmentCount(s1, s2);
  
         std::vector<base::State *> block;
         si_->getMotionStates(s1, s2, block, n - 1, false, true);
         newStates.insert(newStates.end(), block.begin(), block.end());
     }
     newStates.push_back(states_[segments]);
     states_.swap(newStates);
 }
  
 void interpolate(unsigned int requestCount)
 {
     if (requestCount < states_.size() || states_.size() < 2)
         return;
  
     unsigned int count = requestCount;
  
     // the remaining length of the path we need to add states along
     double remainingLength = length();
  
     // the new array of states this path will have
     std::vector<base::State *> newStates;
     const int n1 = states_.size() - 1;
  
     for (int i = 0; i < n1; ++i)
     {
         base::State *s1 = states_[i];
         base::State *s2 = states_[i + 1];
  
         newStates.push_back(s1);
  
         // the maximum number of states that can be added on the current motion (without its endpoints)
         // such that we can at least fit the remaining states
         int maxNStates = count + i - states_.size();
  
         if (maxNStates > 0)
         {
             // compute an approximate number of states the following segment needs to contain; this includes endpoints
             double segmentLength = si_->distance(s1, s2);
             int ns =
                 i + 1 == n1 ? maxNStates + 2 : (int)floor(0.5 + (double)count * segmentLength / remainingLength) + 1;
  
             // if more than endpoints are needed
             if (ns > 2)
             {
                 ns -= 2;  // subtract endpoints
  
                 // make sure we don't add too many states
                 if (ns > maxNStates)
                     ns = maxNStates;
  
                 // compute intermediate states
                 std::vector<base::State *> block;
                 si_->getMotionStates(s1, s2, block, ns, false, true);
                 newStates.insert(newStates.end(), block.begin(), block.end());
             }
             else
                 ns = 0;
  
             // update what remains to be done
             count -= (ns + 1);
             remainingLength -= segmentLength;
         }
         else
             count--;
     }
  
     // add the last state
     newStates.push_back(states_[n1]);
     states_.swap(newStates);
 }
*/
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

ompl::control::STLRRTUnicycle::STLRRTUnicycle(const STLSpaceInformationPtr &stlsi, const STLSpaceInformationPtr &scout_stlsi, ProductGraphPtr a, ProductGraphPtr b, double exploreTime, double scout_exploreTime)
  : ompl::base::Planner(stlsi, "STLRRTUnicycle")
  , scout_stlsi_(scout_stlsi.get())
  , stlsi_(stlsi.get())
  , abstraction_(std::move(a))
  , scout_abstraction_(std::move(b))
  , exploreTime_(exploreTime)
  , scout_exploreTime_(scout_exploreTime)
  , scoutplanner(ScoutPlanner(scout_stlsi))
{
    specs_.approximateSolutions = true;

    std::cout << "WHAT" << std::endl;

    // Planner::declareParam<double>("goal_bias", this, &STLRRTUnicycle::setGoalBias, &STLRRTUnicycle::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("use_scout", this, &STLRRTUnicycle::setUseScout, &STLRRTUnicycle::getUseScout);
    // Planner::declareParam<double>("pruning_radius", this, &STLRRTUnicycle::setPruningRadius, &STLRRTUnicycle::getPruningRadius, "0.:.1:100");
    // Planner::declareParam<double>("traj_bias", this, &STLRRTUnicycle::setTrajBias, &STLRRTUnicycle::setTrajBias, "0.:.05:1.");
}

ompl::control::STLRRTUnicycle::~STLRRTUnicycle()
{
    clearMotions();
}

void ompl::control::STLRRTUnicycle::setup()
{
    base::Planner::setup();
    scoutplanner.setup();
}

void ompl::control::STLRRTUnicycle::clear()
{
    base::Planner::clear();
    availDist_.clear();
    abstractInfo_.clear();
    clearMotions();
}

ompl::control::ScoutPlanner::ScoutPlanner(const STLSpaceInformationPtr &stlsi)
  : ompl::base::Planner(stlsi, "STLScoutPlanner")
{
    std::cout << "completed stl scout planner" << std::endl;
}

ompl::control::ScoutPlanner::~ScoutPlanner() = default;

void ompl::control::ScoutPlanner::setup()
{
    base::Planner::setup();
}

void ompl::control::ScoutPlanner::clear()
{
    // scoutplanner.base::Planner::clear();
    // scout_availDist_.clear();
    // scout_abstractInfo_.clear();
    // clearMotions();
}

ompl::base::PlannerStatus ompl::control::ScoutPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    return {false, false};
}

ompl::base::PlannerStatus ompl::control::STLRRTUnicycle::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // \todo make solve work when called more than once!
    checkValidity();

    // if (use_scout_){
        //TODO: make sure initializing of product graph separately for scout and main planner works!??
        auto pis = scoutplanner.getPlannerInputStates();
        auto *scout_start = pis.nextStart(); //scoutplanner.pis_.nextStart();
        
        scout_prodStart_ = scout_stlsi_->getProdGraphState(scout_start);

        // if (scoutplanner.pis_.haveMoreStartStates())
        //     OMPL_WARN("Multiple start states given. Using only the first start state.");

        auto *scout_startMotion = new Motion(scout_stlsi_);
        scoutplanner.getSpaceInformation()->copyState(scout_startMotion->state, scout_start);
        scout_stlsi_->nullControl(scout_startMotion->control);
        scout_startMotion->abstractState = scout_prodStart_;
        scout_motions_.push_back(scout_startMotion);
        scout_abstractInfo_[scout_prodStart_].addMotion(scout_startMotion);
        if (!scout_abstractInfo_[scout_prodStart_].nn)
        {
            scout_abstractInfo_[scout_prodStart_].nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
            scout_abstractInfo_[scout_prodStart_].nn->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunctionScout(a, b); });
        }
        scout_abstractInfo_[scout_prodStart_].nn->add(scout_startMotion);

        if (!nn_)
        {
            scout_nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
            scout_nn_->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunctionScout(a, b); });
        }
        scout_nn_->add(scout_startMotion);
        updateWeightScout(scout_prodStart_);
        scout_availDist_.add(scout_prodStart_, scout_abstractInfo_[scout_prodStart_].weight);

        scout_abstraction_->buildGraph(scout_prodStart_, [this](ProductGraph::State *as)
                                {
                                    initAbstractInfoScout(as);
                                });

        if (!scout_sampler_)
            scout_sampler_ = scoutplanner.getSpaceInformation()->allocStateSampler();
        if (!scout_controlSampler_)
            scout_controlSampler_ = scout_stlsi_->allocControlSampler();
    // }
    ////////////////
    //MAIN PLANNER//
    ////////////////
    const base::State *start = pis_.nextStart();
    prodStart_ = stlsi_->getProdGraphState(start);

    // if (pis_.haveMoreStartStates())
        // OMPL_WARN("Multiple start states given. Using only the first start state.");

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

    // if (!abstractInfo_[prodStart_].witnesses)
    // {
    //     abstractInfo_[prodStart_].witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    //     abstractInfo_[prodStart_].witnesses->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunction(a, b); });
    // }
    // abstractInfo_[prodStart_].witnesses->add(startMotion);

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
        controlSampler_ = stlsi_->allocDirectedControlSampler();

    bool solved = false;
    bool scout_solved = false;
    Motion *soln;
    Motion *scout_path;
    int i = 0;
    double radius_ = 7.0; // this includes all orientation and velocities range
    int bias_type = 0;
    double bias_p_ = 0.05; //0.01; //-5 ;//0.1; //-5; //0.05; //0.2
    bool scout_lead_true = false;
    while (ptc() == false && !solved)
    {
        ++i;
        
        // if (i > 10){
        //     i = 0;
        //     availDist_.clear();
        //     abstractInfo_.clear();
        //     prodStart_ = stlsi_->getProdGraphState(start);

        //     // if (pis_.haveMoreStartStates())
        //         // OMPL_WARN("Multiple start states given. Using only the first start state.");

        //     auto *startMotion = new Motion(stlsi_);
        //     si_->copyState(startMotion->state, start);
        //     stlsi_->nullControl(startMotion->control);
        //     startMotion->abstractState = prodStart_;
        //     motions_.push_back(startMotion);
        //     abstractInfo_[prodStart_].addMotion(startMotion);
        //     if (!abstractInfo_[prodStart_].nn)
        //     {
        //         abstractInfo_[prodStart_].nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        //         abstractInfo_[prodStart_].nn->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunction(a, b); });
        //     }
        //     abstractInfo_[prodStart_].nn->add(startMotion);

        //     // if (!abstractInfo_[prodStart_].witnesses)
        //     // {
        //     //     abstractInfo_[prodStart_].witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        //     //     abstractInfo_[prodStart_].witnesses->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunction(a, b); });
        //     // }
        //     // abstractInfo_[prodStart_].witnesses->add(startMotion);

        //     if (!nn_)
        //     {
        //         nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        //         nn_->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunction(a, b); });
        //     }
        //     nn_->add(startMotion);
        //     updateWeight(prodStart_);
        //     availDist_.add(prodStart_, abstractInfo_[prodStart_].weight);

        //     abstraction_->buildGraph(prodStart_, [this](ProductGraph::State *as)
        //                             {
        //                                 initAbstractInfo(as);
        //                             });
        // }

        if (use_scout_){
            
            
            if (rng_.uniform01() < 0.5 && scout_solved){
            //         scout_motions_.clear();
            //         scout_motions_.push_back(scout_startMotion);
            //     }
            // scout_abstractInfo_.clear();
            
            scout_availDist_.clear();
            scout_abstractInfo_.clear();
            // for (auto info:scout_abstractInfo_){
            //     info.second.motions.clear();
            //     info.second.motionElems.clear();
            //     info.second.pdfElem = nullptr;
            //     info.second.nn->clear();
            // }
            
            scout_prodStart_ = scout_stlsi_->getProdGraphState(scout_start);
            auto *scout_startMotion = new Motion(scout_stlsi_);
            scoutplanner.getSpaceInformation()->copyState(scout_startMotion->state, scout_start);
            scout_stlsi_->nullControl(scout_startMotion->control);
            scout_startMotion->abstractState = scout_prodStart_;
            scout_abstractInfo_[scout_prodStart_].addMotion(scout_startMotion);
            // scout_abstractInfo_[scout_prodStart_].nn->add(scout_startMotion);

            if (!scout_abstractInfo_[scout_prodStart_].nn)
            {
                scout_abstractInfo_[scout_prodStart_].nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
                scout_abstractInfo_[scout_prodStart_].nn->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunctionScout(a, b); });
            }
            scout_abstractInfo_[scout_prodStart_].nn->add(scout_startMotion);

            updateWeightScout(scout_prodStart_);
            scout_availDist_.add(scout_prodStart_, scout_abstractInfo_[scout_prodStart_].weight);
            scout_abstraction_->buildGraph(scout_prodStart_, [this](ProductGraph::State *as)
                                {
                                    initAbstractInfoScout(as);
                                });
            }
            
            const std::vector<ProductGraph::State *> lead =
                scout_abstraction_->computeLead(scout_prodStart_, [this](ProductGraph::State *a, ProductGraph::State *b)
                                        {
                                            return abstractEdgeWeightScout(a, b);
                                        });
            for (auto l : lead){
                std::cout << l->getCosafeState() << " " << l->getDecompRegion() << ", " ;
            }
            std::cout << std::endl;

            buildAvailScout(lead);
            // scout_availDist_.add(scout_prodStart_, scout_abstractInfo_[scout_prodStart_].weight);

            scout_solved = scout(lead, scout_path, scout_exploreTime_);
            std::vector<ob::State*> lead_path;
            std::unordered_map<int, std::vector<ob::State* >> lead_path_automaton;
            std::vector<int> scout_lead; //TODO: fix problems here
            // scout_solved = false;
            if (scout_solved){
                OMPL_INFORM("Found Scout Solution: Finding Main Solution NOW");
                // if (rng_.uniform01() < 0.1){
                //     motions_.clear();
                //     motions_.push_back(startMotion);
                // }
                samplers_.clear();

                const std::vector<ProductGraph::State *> lead_uni =
                abstraction_->computeLead(prodStart_, [this](ProductGraph::State *a, ProductGraph::State *b)
                                        {
                                            return abstractEdgeWeight(a, b);
                                        });

                for (auto l : lead_uni){
                std::cout << l->getCosafeState() << " " << l->getDecompRegion() << ", " ;
                }
                std::cout << std::endl;
                buildAvail(lead_uni);
                
                // ProductGraphStateInfo &info = abstractInfo_[prodStart_];
                // info.pdfElem = availDist_.add(prodStart_, info.weight);
                
                std::vector<Motion *> path;
                int prev_val = -1;
                int new_val;
                int index = 0;
                while (scout_path != nullptr)
                {
                    path.push_back(scout_path);
                    scout_path = scout_path->parent;
                    
                }
                // OMPL_INFORM("Here");
                std::vector<ob::State*> statesToSample;
                sampler_.reset();
                si_->getStateSpace()->clearStateSamplerAllocator();
                sampler_ = si_->getStateSpace()->allocStateSampler();
                int prev_cs = 100;
                for (int i = path.size() - 2; i >= 0; --i) //for (int i = 0; i < path.size() - 1; ++i)
                {
                    ob::State *s = si_->allocState();
                    sampler_->sampleUniform(s);
                    // change geometric part (R2)
                    R2BeliefSpace::StateType *from = path[i]->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>();
                    s->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setXY(from->values[0], from->values[1]);
                    // s->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setXY(from->values[0], from->values[1]);
                    // update the lead path
                    std::cout << from->values[0] << " " << from->values[1] << " " << from->getCovariance().trace() << " " << path[i]->abstractState->getCosafeState()<< std::endl;
                    // lead_path.push_back(s);
                    int cs = path[i]->abstractState->getCosafeState();
                    
                    // if (from->values[1] < -43 && path[i]->abstractState->getCosafeState() != 3){
                    // //     lead_path_automaton[cs].push_back(s);
                    //     std::cout << from->values[0] << " " << from->values[1] << " " << path[i]->abstractState->getCosafeState()<< std::endl;
                    // }
                    if (prev_cs != cs && prev_cs != 100){
                        lead_path_automaton[prev_cs].push_back(s);
                        // std::cout << from->values[0] << " " << from->values[1] << " " << prev_cs<< std::endl;
                    }
                    // if (prev_cs != cs && prev_cs != 100){
                    //     for (int i = 1; i < 11; ++i){
                    //         ob::State *new_s = si_->allocState();
                    //         sampler_->sampleUniform(new_s);
                    //         if (cs == 1)
                    //         new_s->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setXY(from->values[0], from->values[1]);
                    //         lead_path_automaton[prev_cs].push_back(s);
                    //     }
                    //     lead_path_automaton[cs].push_back(lead_path_automaton[prev_cs].back());
                        // std::cout << from->values[0] << " " << from->values[1] << " " << from->getCovariance().trace() << " " << path[i]->abstractState->getCosafeState()<< std::endl;
                    //     //  lead_path_automaton[cs].push_back(lead_path_automaton[prev_cs].back());
                    //     //   lead_path_automaton[cs].push_back(lead_path_automaton[prev_cs].back());
                    //     //    lead_path_automaton[cs].push_back(lead_path_automaton[prev_cs].back());
                    // }
                    else{
                        lead_path_automaton[cs].push_back(s);
                    }
                    prev_cs = cs;
                }
                // feed the leads
                for (auto &l:lead_path_automaton){
                    int cs = l.first;
                    std::vector<ob::State*> l_path = l.second;
                    si_->getStateSpace()->setStateSamplerAllocator(std::bind(&AllocGuidedStateSampler, &(*si_->getStateSpace()), l_path, radius_, bias_type, bias_p_));
                    samplers_[cs] = si_->getStateSpace()->allocStateSampler();
                }
                scout_lead_true = true;
                // std::cout << "exploring" << std::endl;
                solved = explore(lead_uni, scout_lead_true, soln, exploreTime_); //TODO:add scout path
                
            }
            else{
                OMPL_INFORM("Cannot find Scout Solution");
                const std::vector<ProductGraph::State *> lead_uni =
                abstraction_->computeLead(prodStart_, [this](ProductGraph::State *a, ProductGraph::State *b)
                                        {
                                            return abstractEdgeWeight(a, b);
                                        });
                buildAvail(lead_uni);
                //  std::cout << "exploring 2" << std::endl;
                solved = explore(lead_uni, scout_lead_true, soln, exploreTime_);
            }
        }
        else{
            
            const std::vector<ProductGraph::State *> lead_uni =
                abstraction_->computeLead(prodStart_, [this](ProductGraph::State *a, ProductGraph::State *b)
                                        {
                                            return abstractEdgeWeight(a, b);
                                        });
                buildAvail(lead_uni);

            for (auto l : lead_uni){
                std::cout << l->getCosafeState() << " " << l->getDecompRegion() << ", " ;
                }
                std::cout << std::endl;
                solved = explore(lead_uni, scout_lead_true, soln, exploreTime_);
        }
    }

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
    OMPL_INFORM("Created %u states", motions_.size());
    num_vertices_ = motions_.size();
    return {solved, false};
}

void ompl::control::STLRRTUnicycle::getTree(std::vector<base::State *> &tree) const
{
    tree.resize(motions_.size());
    for (unsigned int i = 0; i < motions_.size(); ++i)
        tree[i] = motions_[i]->state;
}

std::vector<ompl::control::ProductGraph::State *>
ompl::control::STLRRTUnicycle::getHighLevelPath(const std::vector<base::State *> &path, ProductGraph::State *start) const
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

ompl::control::STLRRTUnicycle::Motion::Motion(const SpaceInformation *si)
  : state(si->allocState()), control(si->allocControl())
{
}

ompl::control::STLRRTUnicycle::Motion::~Motion() = default;


void ompl::control::STLRRTUnicycle::ProductGraphStateInfo::addMotion(Motion *m)
{
    motionElems[m] = motions.add(m, 1.);
}

double ompl::control::STLRRTUnicycle::updateWeightScout(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = scout_abstractInfo_[as];
    /* \todo weight should include freeVolume, for cases in which decomposition
       does not respect obstacles. */
    info.weight = ((info.motions.size() + 1) * info.volume) / (info.autWeight * (info.numSel + 1) * (info.numSel + 1));
    return info.weight;
}

double ompl::control::STLRRTUnicycle::updateWeight(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = abstractInfo_[as];
    /* \todo weight should include freeVolume, for cases in which decomposition
       does not respect obstacles. */
    info.weight = ((info.motions.size() + 1) * info.volume) / (info.autWeight * (info.numSel + 1) * (info.numSel + 1));
    return info.weight;
}

void ompl::control::STLRRTUnicycle::initAbstractInfoScout(ProductGraph::State *as)
{

    ProductGraphStateInfo &scout_info = scout_abstractInfo_[as];
    scout_info.numSel = 0;
    scout_info.pdfElem = nullptr;
    scout_info.volume = scout_abstraction_->getRegionVolume(as);
    unsigned int autDist = std::max(scout_abstraction_->getCosafeAutDistance(as), scout_abstraction_->getSafeAutDistance(as));
    if (autDist == 0)
        scout_info.autWeight = std::numeric_limits<double>::epsilon();
    else
        scout_info.autWeight = autDist;
    scout_info.weight = scout_info.volume / scout_info.autWeight;
    if (!scout_info.nn)
    {
        scout_info.nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        scout_info.nn->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunctionScout(a, b); });
    }
}

void ompl::control::STLRRTUnicycle::initAbstractInfo(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = abstractInfo_[as];
    info.numSel = 0;
    info.pdfElem = nullptr;
    info.volume = abstraction_->getRegionVolume(as);
    unsigned int autDist = std::max(abstraction_->getCosafeAutDistance(as), abstraction_->getSafeAutDistance(as));
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
    if (!info.witnesses)
    {
        info.witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        info.witnesses->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    }
}

void ompl::control::STLRRTUnicycle::buildAvail(const std::vector<ProductGraph::State *> &lead)
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
        if (rng_.uniform01() < 0.5)
                break;
        }
    }
}

void ompl::control::STLRRTUnicycle::buildAvailScout(const std::vector<ProductGraph::State *> &lead)
{
    for (unsigned int i = 0; i < scout_availDist_.size(); ++i)
        scout_abstractInfo_[scout_availDist_[i]].pdfElem = nullptr;
    scout_availDist_.clear();
    unsigned int numTreePts = 1;
    bool empty = false;
    for (int i = lead.size() - 1; i >= 0; --i)
    {
        ProductGraph::State *as = lead[i];
        ProductGraphStateInfo &info = scout_abstractInfo_[as];
        if (!info.motions.empty())
        {
        info.pdfElem = scout_availDist_.add(as, info.weight);
        numTreePts += info.motions.size();
        if (rng_.uniform01() < 0.5)
                break;
        }
    }
}

bool ompl::control::STLRRTUnicycle::scout(const std::vector<ProductGraph::State *> &lead, Motion *&soln, double duration)
{
    bool solved = false;
    base::PlannerTerminationCondition ptc = base::timedPlannerTerminationCondition(duration);
    base::GoalPtr goal = scoutplanner.getProblemDefinition()->getGoal();
    auto *rmotion = new Motion(scout_stlsi_);
    base::State *rstate = rmotion->state;
    while (!ptc() && !solved)
    {
        ProductGraph::State *as = scout_availDist_.sample(rng_.uniform01());
        // OMPL_INFORM("as");
        ++scout_abstractInfo_[as].numSel;
        updateWeightScout(as);
        // OMPL_INFORM("UPDATED WEIGHT");
        PDF<Motion *> &motions = scout_abstractInfo_[as].motions;
        Motion *v;
        scout_sampler_->sampleUniform(rstate);
        if (rng_.uniform01() < 0.1){ //sampling_bias_;
            rstate->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setSigma(0.5);
        }
        else{
            rstate->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->setSigma(30.0*rng_.uniform01());
        }
        v = scout_abstractInfo_[as].nn->nearest(rmotion);
        // OMPL_INFORM("GOT V");
        if (!motions.empty()){
            PDF<Motion *>::Element *velem = scout_abstractInfo_[as].motionElems[v];
        double vweight = motions.getWeight(velem);
        if (vweight > 1e-20)
            motions.update(velem, vweight / (vweight + 1.));
        }
        Control *rctrl = scout_stlsi_->allocControl();
        scout_controlSampler_->sampleNext(rctrl, v->control, v->state);
        unsigned int cd =
            scout_controlSampler_->sampleStepCount(scout_stlsi_->getMinControlDuration(), scout_stlsi_->getMaxControlDuration());

        base::State *newState = scoutplanner.getSpaceInformation()->allocState();
        cd = scout_stlsi_->propagateWhileValid(v->state, rctrl, cd, newState);

        // OMPL_INFORM("propagating");

        if (cd < scout_stlsi_->getMinControlDuration())
        {
            scoutplanner.getSpaceInformation()->freeState(newState);
            scout_stlsi_->freeControl(rctrl);
            continue;
        }
        // OMPL_INFORM("adding to motion");
        auto *m = new Motion();
        m->state = newState;
        m->control = rctrl;
        m->steps = cd;
        m->parent = v;
        // Since the state was determined to be valid by SpaceInformation, we don't need to check automaton states
        // OMPL_INFORM("getprodgraphstate");
        m->abstractState = scout_stlsi_->getProdGraphState(m->state);

        // std::cout << scout_stlsi_->getProdGraphState(m->state)->getDecompRegion() << std::endl;
        // OMPL_INFORM("gotten");
        scout_motions_.push_back(m);
        // std::cout << m->abstractState->getSafeState() << " " << m->abstractState->getCosafeState() << " " << m->abstractState->getDecompRegion() << std::endl;
        scout_abstractInfo_[m->abstractState].addMotion(m);
        scout_abstractInfo_[m->abstractState].nn->add(m);
        // scout_nn_->add(m);
        updateWeightScout(m->abstractState);
        // OMPL_INFORM("huh");
        // update weight if hl state already exists in avail
        if (scout_abstractInfo_[m->abstractState].pdfElem != nullptr)
            scout_availDist_.update(scout_abstractInfo_[m->abstractState].pdfElem, scout_abstractInfo_[m->abstractState].weight);
        else
        {
            // otherwise, only add hl state to avail if it already exists in lead
            if (std::find(lead.begin(), lead.end(), m->abstractState) != lead.end())
            {
                PDF<ProductGraph::State *>::Element *elem =
                    scout_availDist_.add(m->abstractState, scout_abstractInfo_[m->abstractState].weight);
                scout_abstractInfo_[m->abstractState].pdfElem = elem;
            }
        }
        solved = goal->isSatisfied(m->state);
        if (solved)
        {
            soln = m;
            break;
        }
    }

    if (rmotion->state)
            scoutplanner.getSpaceInformation()->freeState(rmotion->state);
        if (rmotion->control)
            scout_stlsi_->freeControl(rmotion->control);
        delete rmotion;
    
    return solved;
}
/*
bool ompl::control::STLRRTUnicycle::explore(const std::vector<ProductGraph::State *> &lead, Motion *&scout_path, Motion *&soln, double duration)
{
    bool solved = false;
    base::PlannerTerminationCondition ptc = base::timedPlannerTerminationCondition(duration);
    base::GoalPtr goal = pdef_->getGoal();

    // base::Goal *goal_is = pdef_->getGoal().get();
    // auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal_is);

    //TODO: check how to project onto the same product graph
    auto *rmotion = new Motion(stlsi_);
    base::State *rstate = rmotion->state;
    while (!ptc() && !solved)
    {
        Motion *v;
        sampler_->sampleUniform(rstate);
        if (rng_.uniform01() < 0.2){ //sampling_bias_;
            rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigma(0.5);
        }
        else{
            // double val = rng_.uniform01();
            rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigmaX(2*rng_.uniform01());
            rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigmaY(2*rng_.uniform01());
        }

        // if (rng_.uniform01() < 0.05){
        //     rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setX(95.0);
        //     rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setY(95.0);
        // }
        v = nn_->nearest(rmotion);
        Control *rctrl = stlsi_->allocControl();
        // controlSampler_->sampleNext(rctrl, v->control, v->state);
        // unsigned int cd =
        //     controlSampler_->sampleStepCount(stlsi_->getMinControlDuration(), stlsi_->getMaxControlDuration());
        base::State *newState = si_->allocState();
        unsigned int cd = controlSampler_->sampleTo(rctrl, v->control, v->state, newState);
        
        // cd = stlsi_->propagateWhileValid(v->state, rctrl, cd, newState);
        if (cd < stlsi_->getMinControlDuration())
        {
            si_->freeState(newState);
            stlsi_->freeControl(rctrl);
            continue;
        }
        auto *m = new Motion();
        m->state = newState;
        m->control = rctrl;
        m->steps = cd;
        m->parent = v;
        // Since the state was determined to be valid by SpaceInformation, we don't need to check automaton states
        m->abstractState = stlsi_->getProdGraphState(m->state);
        motions_.push_back(m);
        nn_->add(m);
        solved = goal->isSatisfied(m->state);
        if (solved)
        {
            soln = m;
            break;
        }
    }

    if (rmotion->state)
            si_->freeState(rmotion->state);
        if (rmotion->control)
            stlsi_->freeControl(rmotion->control);
        delete rmotion;
    
    return solved;
}
*/
bool ompl::control::STLRRTUnicycle::explore(const std::vector<ProductGraph::State *> &lead, bool scout_lead_true, Motion *&soln, double duration)
{
    bool solved = false;
    base::PlannerTerminationCondition ptc = base::timedPlannerTerminationCondition(duration);
    base::GoalPtr goal = pdef_->getGoal();

    //TODO: check how to project onto the same product graph
    auto *rmotion = new Motion(stlsi_);
    base::State *rstate = rmotion->state;
    while (!ptc() && !solved)
    {
        ProductGraph::State *as = availDist_.sample(rng_.uniform01());
        ++abstractInfo_[as].numSel;
        updateWeight(as);
        PDF<Motion *> &motions = abstractInfo_[as].motions;
        
        Motion *v;
        if (scout_lead_true)
            samplers_[as->getCosafeState()]->sampleUniform(rstate);
        else
            sampler_->sampleUniform(rstate);
        if (rng_.uniform01() < 0.2){ //sampling_bias_;
            rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigma(0.5);
        }
        else{
            // double val = rng_.uniform01();
            rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigmaX(5.0*rng_.uniform01());
            rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigmaY(5.0*rng_.uniform01());
        }

        // if (rng_.uniform01() < 0.05){
        //     rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setX(95.0);
        //     rstate->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setY(95.0);
        // }
        // v = nn_->nearest(rmotion);
        v = abstractInfo_[as].nn->nearest(rmotion);
        Control *rctrl = stlsi_->allocControl();
        // controlSampler_->sampleNext(rctrl, v->control, v->state);
        // unsigned int cd =
        //     controlSampler_->sampleStepCount(stlsi_->getMinControlDuration(), stlsi_->getMaxControlDuration());
        base::State *newState = si_->allocState();
        unsigned int cd = controlSampler_->sampleTo(rctrl, v->control, v->state, newState);
        
        // cd = stlsi_->propagateWhileValid(v->state, rctrl, cd, newState);
        if (cd < stlsi_->getMinControlDuration())
        {
            si_->freeState(newState);
            stlsi_->freeControl(rctrl);
            continue;
        }
        auto *m = new Motion();
        m->state = newState;
        m->control = rctrl;
        m->steps = cd;
        m->parent = v;
        // Since the state was determined to be valid by SpaceInformation, we don't need to check automaton states
        m->abstractState = stlsi_->getProdGraphState(m->state);
        motions_.push_back(m);
        nn_->add(m);
        abstractInfo_[m->abstractState].addMotion(m);
        abstractInfo_[m->abstractState].nn->add(m);

        // if (m->abstractState->getCosafeState() == 1 && m->abstractState->getDecompRegion() == 3){
        //     OMPL_INFORM("REACHED 1");
        // }

        // else if (m->abstractState->getCosafeState() == 2){
        //     OMPL_INFORM("REACHED 2");
        // }

        updateWeight(m->abstractState);

        // update weight if hl state already exists in avail
            if (abstractInfo_[m->abstractState].pdfElem != nullptr)
                availDist_.update(abstractInfo_[m->abstractState].pdfElem, abstractInfo_[m->abstractState].weight);
            else
            {
                if (scout_lead_true){
                    if (samplers_.find(m->abstractState->getCosafeState()) != samplers_.end()){
                        PDF<ProductGraph::State *>::Element *elem =
                        availDist_.add(m->abstractState, abstractInfo_[m->abstractState].weight);
                        abstractInfo_[m->abstractState].pdfElem = elem;
                    }
                }
                // otherwise, only add hl state to avail if it already exists in lead
                else if (std::find(lead.begin(), lead.end(), m->abstractState) != lead.end())
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

    if (rmotion->state)
            si_->freeState(rmotion->state);
        if (rmotion->control)
            stlsi_->freeControl(rmotion->control);
        delete rmotion;
    
    return solved;
}

double ompl::control::STLRRTUnicycle::abstractEdgeWeightScout(ProductGraph::State *a, ProductGraph::State *b) const
{
    const ProductGraphStateInfo &infoA = scout_abstractInfo_.find(a)->second;
    const ProductGraphStateInfo &infoB = scout_abstractInfo_.find(b)->second;
    return 1. / (infoA.weight * infoB.weight);
}

double ompl::control::STLRRTUnicycle::abstractEdgeWeight(ProductGraph::State *a, ProductGraph::State *b) const
{
    const ProductGraphStateInfo &infoA = abstractInfo_.find(a)->second;
    const ProductGraphStateInfo &infoB = abstractInfo_.find(b)->second;
    return 1. / (infoA.weight * infoB.weight);
}

void ompl::control::STLRRTUnicycle::clearMotions()
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