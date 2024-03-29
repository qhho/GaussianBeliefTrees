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

#include "GBT_SST.h"
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

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"


ompl::control::GBTSSTUnicycle::GBTSSTUnicycle(const STLSpaceInformationPtr &stlsi, const STLSpaceInformationPtr &scout_stlsi, ProductGraphPtr a, ProductGraphPtr b, double exploreTime, double scout_exploreTime)
  : ompl::base::Planner(stlsi, "GBTSSTUnicycle")
  , scout_stlsi_(scout_stlsi.get())
  , stlsi_(stlsi.get())
  , abstraction_(std::move(a))
  , scout_abstraction_(std::move(b))
  , exploreTime_(exploreTime)
  , scout_exploreTime_(scout_exploreTime)
  , scoutplanner(SSTScoutPlanner(scout_stlsi))
{
    specs_.approximateSolutions = true;

    // Planner::declareParam<double>("goal_bias", this, &GBTSSTUnicycle::setGoalBias, &GBTSSTUnicycle::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("use_scout", this, &GBTSSTUnicycle::setUseScout, &GBTSSTUnicycle::getUseScout);
    Planner::declareParam<double>("pruning_radius", this, &GBTSSTUnicycle::setPruningRadius, &GBTSSTUnicycle::getPruningRadius, "0.:.1:100");
    // // Planner::declareParam<double>("traj_bias", this, &GBTSSTUnicycle::setTrajBias, &GBTSSTUnicycle::setTrajBias, "0.:.05:1.");
    Planner::declareParam<double>("selection_radius", this, &GBTSSTUnicycle::setSelectionRadius, &GBTSSTUnicycle::getSelectionRadius, "0.:.1:"
                                                                                                                "100");
}

ompl::control::GBTSSTUnicycle::~GBTSSTUnicycle()
{
    clearMotions();
}

void ompl::control::GBTSSTUnicycle::setup()
{
    base::Planner::setup();
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
        }
        else{
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }
    std::cout << "AT HERE" << std::endl;
    scout_prevSolutionCost_ = opt_->infiniteCost();
    scoutplanner.setup();
}

void ompl::control::GBTSSTUnicycle::clear()
{
    base::Planner::clear();
    availDist_.clear();
    abstractInfo_.clear();
    if (opt_)
        scout_prevSolutionCost_ = opt_->infiniteCost();
    
    clearMotions();
}

ompl::control::SSTScoutPlanner::SSTScoutPlanner(const STLSpaceInformationPtr &stlsi)
  : ompl::base::Planner(stlsi, "STLSSTScoutPlanner")
{
    std::cout << "completed stl scout planner" << std::endl;
}

ompl::control::SSTScoutPlanner::~SSTScoutPlanner() = default;

void ompl::control::SSTScoutPlanner::setup()
{
    base::Planner::setup();
}

void ompl::control::SSTScoutPlanner::clear()
{

    
    // SSTScoutPlanner.base::Planner::clear();
    // scout_availDist_.clear();
    // scout_abstractInfo_.clear();
    // clearMotions();
}

ompl::base::PlannerStatus ompl::control::SSTScoutPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    return {false, false};
}

ompl::base::PlannerStatus ompl::control::GBTSSTUnicycle::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // \todo make solve work when called more than once!
    checkValidity();
    std::cout << "AT HERE" << std::endl;
    // if (use_scout_){
        //TODO: make sure initializing of product graph separately for scout and main planner works!??
    auto pis = scoutplanner.getPlannerInputStates();
    auto *scout_start = pis.nextStart(); //SSTScoutPlanner.pis_.nextStart();
    
    scout_prodStart_ = scout_stlsi_->getProdGraphState(scout_start);

    // if (SSTScoutPlanner.pis_.haveMoreStartStates())
    //     OMPL_WARN("Multiple start states given. Using only the first start state.");
    std::cout << "AT HERE" << std::endl;
    auto *scout_startMotion = new Motion(scout_stlsi_);
    scoutplanner.getSpaceInformation()->copyState(scout_startMotion->state, scout_start);
    scout_stlsi_->nullControl(scout_startMotion->control);
    scout_startMotion->abstractState = scout_prodStart_;
    scout_motions_.push_back(scout_startMotion);
    scout_abstractInfo_[scout_prodStart_].addMotion(scout_startMotion);

    std::cout << "AT HERE" << std::endl;

    if (!scout_abstractInfo_[scout_prodStart_].nn)
    {
        scout_abstractInfo_[scout_prodStart_].nn.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        scout_abstractInfo_[scout_prodStart_].nn->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunctionScout(a, b); });
    }
    scout_abstractInfo_[scout_prodStart_].nn->add(scout_startMotion);

    if (!scout_abstractInfo_[scout_prodStart_].witnesses)
    {
        scout_abstractInfo_[scout_prodStart_].witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        scout_abstractInfo_[scout_prodStart_].witnesses->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunctionEuclidean(a, b); });
    }
    // scout_abstractInfo_[scout_prodStart_].witnesses->add(scout_startMotion);

    std::cout << "AT HERE" << std::endl;
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

    OMPL_INFORM("RUNNING MAIN PLANNER");
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

        // OMPL_INFORM("RUNNING ONCE");
        
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
            
            /*
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

            if (!scout_abstractInfo_[scout_prodStart_].witnesses)
            {
                scout_abstractInfo_[scout_prodStart_].witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
                scout_abstractInfo_[scout_prodStart_].witnesses->setDistanceFunction([this](Motion *a, Motion *b) { return distanceFunctionScout(a, b); });
            }
            // scout_abstractInfo_[scout_prodStart_].witnesses->add(scout_startMotion);

            updateWeightScout(scout_prodStart_);
            scout_availDist_.add(scout_prodStart_, scout_abstractInfo_[scout_prodStart_].weight);
            scout_abstraction_->buildGraph(scout_prodStart_, [this](ProductGraph::State *as)
                                {
                                    initAbstractInfoScout(as);
                                });
            }
            */
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

            // if (!scout_solved)
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

void ompl::control::GBTSSTUnicycle::getTree(std::vector<base::State *> &tree) const
{
    tree.resize(motions_.size());
    for (unsigned int i = 0; i < motions_.size(); ++i)
        tree[i] = motions_[i]->state;
}

std::vector<ompl::control::ProductGraph::State *>
ompl::control::GBTSSTUnicycle::getHighLevelPath(const std::vector<base::State *> &path, ProductGraph::State *start) const
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

ompl::control::GBTSSTUnicycle::Motion::Motion(const SpaceInformation *si)
  : state(si->allocState()), control(si->allocControl())
{
}

ompl::control::GBTSSTUnicycle::Motion::~Motion() = default;


void ompl::control::GBTSSTUnicycle::ProductGraphStateInfo::addMotion(Motion *m)
{
    motionElems[m] = motions.add(m, 1.);
}

double ompl::control::GBTSSTUnicycle::updateWeightScout(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = scout_abstractInfo_[as];
    /* \todo weight should include freeVolume, for cases in which decomposition
       does not respect obstacles. */
    info.weight = ((info.motions.size() + 1) * info.volume) / (info.autWeight * (info.numSel + 1) * (info.numSel + 1));
    return info.weight;
}

double ompl::control::GBTSSTUnicycle::updateWeight(ProductGraph::State *as)
{
    ProductGraphStateInfo &info = abstractInfo_[as];
    /* \todo weight should include freeVolume, for cases in which decomposition
       does not respect obstacles. */
    info.weight = ((info.motions.size() + 1) * info.volume) / (info.autWeight * (info.numSel + 1) * (info.numSel + 1));
    return info.weight;
}

void ompl::control::GBTSSTUnicycle::initAbstractInfoScout(ProductGraph::State *as)
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
    if (!scout_info.witnesses)
    {
        scout_info.witnesses.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        scout_info.witnesses->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunctionEuclidean(a, b); });
    }
}

void ompl::control::GBTSSTUnicycle::initAbstractInfo(ProductGraph::State *as)
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

void ompl::control::GBTSSTUnicycle::buildAvail(const std::vector<ProductGraph::State *> &lead)
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

void ompl::control::GBTSSTUnicycle::buildAvailScout(const std::vector<ProductGraph::State *> &lead)
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

bool ompl::control::GBTSSTUnicycle::scout(const std::vector<ProductGraph::State *> &lead, Motion *&soln, double duration)
{

    // Motion *solution = nullptr;
    // Motion *approxsol = nullptr;
    bool sufficientlyShort = false;
    double approxdif = std::numeric_limits<double>::infinity();

    bool solved = false;
    bool new_solved = false;
    base::PlannerTerminationCondition ptc = base::timedPlannerTerminationCondition(duration);
    base::GoalPtr goal = scoutplanner.getProblemDefinition()->getGoal();
    auto *rmotion = new Motion(scout_stlsi_);
    base::State *rstate = rmotion->state;
    // while (!ptc() && !solved)
    while (!ptc())
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
        rmotion->abstractState = as;
        // v = scout_abstractInfo_[as].nn->nearest(rmotion);
        v = selectNode(rmotion);
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

        // std::cout << rctrl->as<oc::RealVectorControlSpace::ControlType>()->values[0] << std::endl;
        // std::cout << v->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX() << " " << std::endl;
        // std::cout << newState->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX() << " " << newState->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY() << std::endl;

        // OMPL_INFORM("propagating");

        if (cd < scout_stlsi_->getMinControlDuration())
        {
            scoutplanner.getSpaceInformation()->freeState(newState);
            scout_stlsi_->freeControl(rctrl);
            continue;
        }

        scout_stlsi_->copyControl(rmotion->control, rctrl);
        scoutplanner.getSpaceInformation()->copyState(rmotion->state, newState);
        rmotion->abstractState = scout_stlsi_->getProdGraphState(rmotion->state);
        Witness *closestWitness = findClosestWitness(rmotion);

        // std::cout << closestWitness->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX() << closestWitness->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY() << std::endl;

        // std::cout << "here" << std::endl;
        // base::Cost incCost = opt_->motionCost(v->state->as<CompoundState>()->operator[](0), rstate);
        base::Cost incCost = ob::Cost(distanceFunctionScout(v, rmotion));
        // std::cout << "here2" << std::endl;
        base::Cost cost = opt_->combineCosts(v->accCost_, incCost);
        // std::cout << newState->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX() << " " << newState->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY() << std::endl;
        // std::cout << incCost << " " << cost << std::endl;

        if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
        {
            Motion *oldRep = closestWitness->rep_;
            // OMPL_INFORM("adding to motion");
            auto *m = new Motion();
            m->state = newState;
            m->control = rctrl;
            m->steps = cd;
            m->parent = v;
            m->accCost_ = cost;
            v->numChildren_++;
            // OMPL_INFORM("getprodgraphstate");
            m->abstractState = scout_stlsi_->getProdGraphState(m->state);

            closestWitness->linkRep(m);
            scout_motions_.push_back(m);
            // std::cout << m->abstractState->getSafeState() << " " << m->abstractState->getCosafeState() << " " << m->abstractState->getDecompRegion() << std::endl;
            scout_abstractInfo_[m->abstractState].addMotion(m);
            scout_abstractInfo_[m->abstractState].nn->add(m);
            updateWeightScout(m->abstractState);

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

            double dist = 0.0;

            solved = goal->isSatisfied(m->state);

            if (solved && opt_->isCostBetterThan(m->accCost_, scout_prevSolutionCost_))
            {
                soln = m;
                scout_prevSolutionCost_ = soln->accCost_;
                OMPL_INFORM("Found solution with cost %.2f", soln->accCost_.value());
                new_solved = true;
                    // sufficientlyShort = opt_->isSatisfied(soln->accCost_);
                    // if (sufficientlyShort)
                        // break;
                    // if (opt_->isCostBetterThan(soln->accCost_, ob::Cost(200.0))){
                    //     break;
                    // }
            }
            else if (solved && opt_->isCostBetterThan(m->accCost_, scout_prevSolutionCost_)){
                break;
                // std::cout << solved << " " << m->accCost_ << " " << scout_prevSolutionCost_ << std::endl;
            }

            if (oldRep != rmotion)
                {
                    // std::cout << "we have reached here!!!" << std::endl;
                    while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                    {
                        oldRep->inactive_ = true;
                        scout_abstractInfo_[oldRep->abstractState].nn->remove(oldRep);

                        if (oldRep->state)
                            scoutplanner.getSpaceInformation()->freeState(oldRep->state);
                        if (oldRep->control)
                            scout_stlsi_->freeControl(oldRep->control);

                        oldRep->state = nullptr;
                        oldRep->control = nullptr;
                        oldRep->parent->numChildren_--;
                        Motion *oldRepParent = oldRep->parent;
                        delete oldRep;
                        oldRep = oldRepParent;
                    }
                }
        }

        // if (solved)
        // {
        //     soln = m;
        //     break;
        // }
    }

    if (rmotion->state)
            scoutplanner.getSpaceInformation()->freeState(rmotion->state);
        if (rmotion->control)
            scout_stlsi_->freeControl(rmotion->control);
        delete rmotion;
    
    return new_solved; //solved;
}
/*
bool ompl::control::GBTSSTUnicycle::explore(const std::vector<ProductGraph::State *> &lead, Motion *&scout_path, Motion *&soln, double duration)
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
bool ompl::control::GBTSSTUnicycle::explore(const std::vector<ProductGraph::State *> &lead, bool scout_lead_true, Motion *&soln, double duration)
{
    bool solved = false;
    // scout_lead_true = false;
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
        // motions_.push_back(m);
        // nn_->add(m);
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
                // otherwise, only add hl state to avail if it already exists in lead
                // if (std::find(lead.begin(), lead.end(), m->abstractState) != lead.end())
                // {
                    PDF<ProductGraph::State *>::Element *elem =
                        availDist_.add(m->abstractState, abstractInfo_[m->abstractState].weight);
                    abstractInfo_[m->abstractState].pdfElem = elem;
                // }
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

double ompl::control::GBTSSTUnicycle::abstractEdgeWeightScout(ProductGraph::State *a, ProductGraph::State *b) const
{
    const ProductGraphStateInfo &infoA = scout_abstractInfo_.find(a)->second;
    const ProductGraphStateInfo &infoB = scout_abstractInfo_.find(b)->second;
    return 1. / (infoA.weight * infoB.weight);
}

double ompl::control::GBTSSTUnicycle::abstractEdgeWeight(ProductGraph::State *a, ProductGraph::State *b) const
{
    const ProductGraphStateInfo &infoA = abstractInfo_.find(a)->second;
    const ProductGraphStateInfo &infoB = abstractInfo_.find(b)->second;
    return 1. / (infoA.weight * infoB.weight);
}

void ompl::control::GBTSSTUnicycle::clearMotions()
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

    if (nn_)
        nn_->clear();
    
}

ompl::control::GBTSSTUnicycle::Witness *ompl::control::GBTSSTUnicycle::findClosestWitness(ompl::control::GBTSSTUnicycle::Motion *node)
{
    // std::cout << scout_abstractInfo_[node->abstractState].witnesses->size() << std::endl;
    if (scout_abstractInfo_[node->abstractState].witnesses->size() > 0)
    {
        auto *closest = static_cast<Witness *>(scout_abstractInfo_[node->abstractState].witnesses->nearest(node));
        if (distanceFunctionEuclidean(closest, node) > pruningRadius_)
        {
            closest = new Witness(scout_stlsi_);
            closest->linkRep(node);
            scoutplanner.getSpaceInformation()->copyState(closest->state, node->state);
            scout_abstractInfo_[node->abstractState].witnesses->add(closest);
        }
        return closest;
    }
    else
    {
        auto *closest = new Witness(scout_stlsi_);
        closest->linkRep(node);
        scoutplanner.getSpaceInformation()->copyState(closest->state, node->state);
        scout_abstractInfo_[node->abstractState].witnesses->add(closest);
        return closest;
    }
}


ompl::control::GBTSSTUnicycle::Motion *ompl::control::GBTSSTUnicycle::selectNode(ompl::control::GBTSSTUnicycle::Motion *sample)
{
    std::vector<Motion *> ret;
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    scout_abstractInfo_[sample->abstractState].nn->nearestR(sample, selectionRadius_, ret);

    for (auto &i : ret)
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)
    {
        
        int k = 1;
        while (selected == nullptr)
        {
            scout_abstractInfo_[sample->abstractState].nn->nearestK(sample, k, ret);
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}