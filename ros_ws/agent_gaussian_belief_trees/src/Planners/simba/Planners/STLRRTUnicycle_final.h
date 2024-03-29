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

#ifndef OMPL_CONTROL_PLANNERS_STL_CosafeSTLRRTUnicycle_
#define OMPL_CONTROL_PLANNERS_STL_CosafeSTLRRTUnicycle_

#include "ompl/control/planners/PlannerIncludes.h"
#include "../stl/STLProductGraph.h"
#include "../stl/STLSpaceInformation.h"
#include "ompl/datastructures/PDF.h"
#include <unordered_map>
#include <map>
#include <vector>
#include "ompl/datastructures/NearestNeighbors.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include "../StateSamplers/guided_state_sampler.hpp"

namespace ompl
{
    namespace control
    {
        /** \anchor cSTLRRT
            \brief A planner for generating system trajectories to satisfy
            a logical specification given by an automaton, the propositions
            of which are defined over a decomposition of the system's state space.

            \todo cite papers */
        class ScoutPlanner : public base::Planner
            {
                public:
                    ScoutPlanner(const STLSpaceInformationPtr &si);

                    /** \brief Clears all memory belonging to this CosafeSTLRRTUnicycle .*/
                    ~ScoutPlanner() override;

                    /** \brief Initializes CosafeSTLRRTUnicycle data structures. */
                    void setup() override;

                    /** \brief Clears all datastructures belonging to this CosafeSTLRRTUnicycle. */
                    void clear() override;

                    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        };

        class CosafeSTLRRTUnicycle : public base::Planner
        {
        public:
            /** \brief Create an CosafeCosafeSTLRRTUnicycle with a given space and product graph.
                Accepts an optional third parameter to control how much time is spent
                promoting low-level tree exploration along a given high-level lead. */
            CosafeSTLRRTUnicycle(const STLSpaceInformationPtr &si, const STLSpaceInformationPtr &scout_si, ProductGraphPtr a, ProductGraphPtr b, double exploreTime = 4.0, double scout_exploreTime = 1.0);

            /** \brief Clears all memory belonging to this CosafeCosafeSTLRRTUnicycle .*/
            ~CosafeSTLRRTUnicycle() override;

            /// @name ompl::base::Planner Interface
            /// @{

            /** \brief Initializes CosafeSTLRRTUnicycle data structures. */
            void setup() override;

            /** \brief Clears all datastructures belonging to this CosafeCosafeSTLRRTUnicycle. */
            void clear() override;

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            void setUseScout(bool use_scout)
            {
                use_scout_ = use_scout;
            }

            bool getUseScout() const
            {
                return use_scout_;
            }
            

            /** \brief Continues solving until a solution is found
                or a given planner termination condition is met.
                Returns true if a solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            /// @}

            /** \brief Helper debug method to access this planner's
                underlying tree of states. */
            void getTree(std::vector<base::State *> &tree) const;

            /** \brief Helper debug method to return the sequence of high-level product
                graph states corresponding to a sequence of low-level continous system states,
                beginning from an optional initial high-level state. */
            std::vector<ProductGraph::State *> getHighLevelPath(const std::vector<base::State *> &path,
                                                                ProductGraph::State *start = nullptr) const;

            int getNumVertices() const
            {
                return num_vertices_;
            }

            ScoutPlanner scoutplanner;

        protected:
            /** \brief Representation of a motion

                A motion contains pointers to its state, its parent motion, and the control
                that was applied to get from its parent to its state. */
            struct Motion
            {
            public:
                /** \brief Default constructor for Motion. */
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control,
                    given a space. */
                Motion(const SpaceInformation *si);

                /** \brief Motion destructor does not clear memory.
                    Deletions should be performed by the CosafeSTLRRTUnicycle. */
                virtual ~Motion();

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The parent motion in the tree */
                Motion *parent{nullptr};

                /** \brief The number of steps for which the control is applied */
                unsigned int steps{0};

                /** \brief The high-level state to which this motion belongs */
                ProductGraph::State *abstractState{nullptr};
            };

            /** \brief A structure to hold measurement information for a high-level state,
                as well as the set of tree motions belonging to that high-level state.
                Exactly one ProductGraphStateInfo will exist for each ProductGraph::State. */
            struct ProductGraphStateInfo
            {
                /** \brief Creates an info object with no measurements and no tree motions. */
                ProductGraphStateInfo() = default;

                ~ProductGraphStateInfo() {
                    nn->clear();
                }

                /** \brief Adds a tree motion to an info object.
                    This method is called whenever a new tree motion is created
                    in the high-level state corresponding to this info object. */
                void addMotion(Motion *m);

                double weight{0.};
                PDF<Motion *> motions;
                std::unordered_map<Motion *, PDF<Motion *>::Element *> motionElems;
                double volume{0.};
                double autWeight{0.};
                unsigned int numSel{0};
                PDF<ProductGraph::State *>::Element *pdfElem{nullptr};

                /** \brief A nearest-neighbors datastructure containing the tree of motions */
                std::shared_ptr<NearestNeighbors<Motion *>> nn;
                
                std::shared_ptr<NearestNeighbors<Motion *>> witnesses;

                /** \brief Set a different nearest neighbors datastructure */
                // template <template <typename T> class NN>
                // void setNearestNeighbors()
                // {
                //     if (nn && nn->size() != 0)
                //         OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                //     clear();
                //     nn = std::make_shared<NN<Motion *>>();
                //     setup();
                // }
            };

            struct CosafeStateInfo
            {
                /** \brief Creates an info object with no measurements and no tree motions. */
                CosafeStateInfo() = default;

                ~CosafeStateInfo() {
                    nn->clear();
                }

                /** \brief Adds a tree motion to an info object.
                    This method is called whenever a new tree motion is created
                    in the high-level state corresponding to this info object. */
                void addMotion(Motion *m);

                double weight{0.};
                PDF<Motion *> motions;
                std::unordered_map<Motion *, PDF<Motion *>::Element *> motionElems;
                double volume{0.};
                double autWeight{0.};
                unsigned int numSel{0};
                PDF<int >::Element *pdfElem{nullptr};

                /** \brief A nearest-neighbors datastructure containing the tree of motions */
                std::shared_ptr<NearestNeighbors<Motion *>> nn;
                
                std::shared_ptr<NearestNeighbors<Motion *>> witnesses;

                /** \brief Set a different nearest neighbors datastructure */
                // template <template <typename T> class NN>
                // void setNearestNeighbors()
                // {
                //     if (nn && nn->size() != 0)
                //         OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                //     clear();
                //     nn = std::make_shared<NN<Motion *>>();
                //     setup();
                // }
            };

            class Witness : public Motion
            {
            public:
                Witness() = default;

                Witness(const SpaceInformation *si) : Motion(si)
                {
                }
                base::State *getState() const
                {
                    return rep_->state;
                }
                Motion *getParent() const
                {
                    return rep_->parent;
                }

                void linkRep(Motion *lRep)
                {
                    rep_ = lRep;
                }

                /** \brief The node in the tree that is within the pruning radius.*/
                Motion *rep_{nullptr};
            };

            Witness *findClosestWitness(Motion *node);

            /**
                \brief Set the radius for pruning nodes.

                This is the radius used to surround nodes in the witness set.
                Within this radius around a state in the witness set, only one
                active tree node can exist. This limits the size of the tree and
                forces computation to focus on low path costs nodes. If this value
                is too large, narrow passages will be impossible to traverse. In addition,
                children nodes may be removed if they are not at least this distance away
                from their parent nodes.*/
            void setPruningRadius(double pruningRadius)
            {
                pruningRadius_ = pruningRadius;
            }

            /** \brief Get the pruning radius the planner is using */
            double getPruningRadius() const
            {
                return pruningRadius_;
            }

            /** \brief Updates and returns the weight of an abstraction state. */
            virtual double updateWeight(ProductGraph::State *as);

            /** \brief Updates and returns the weight of an abstraction state. */
            virtual double updateWeightScout(int cs);

            /** \brief Initializes the info object for a new high-level state. */
            virtual void initAbstractInfo(ProductGraph::State *as);

            /** \brief Initializes the info object for a new high-level state. */
            virtual void initAbstractInfoScout(int cs);

            /** \brief Compute a set of high-level states along a lead
                to be considered for expansion. */
            virtual void buildAvail(const std::vector<ProductGraph::State *> &lead);

            virtual void buildAvailScout(const std::vector<int > lead);

            /** \brief Expand the tree of motions along a given lead
                for a given duration of time.
                Returns true if a solution was found, in which case the endpoint
                of the solution trajectory will be stored in the given Motion pointer.
                Otherwise, returns false. */
            virtual bool explore(const std::vector<ProductGraph::State *> &lead, bool scout_lead_true, Motion *&soln, double duration);

            virtual bool scout(const std::vector<int > lead, Motion *&soln, double duration);

            /** \brief Returns the weight of an edge between two given high-level states,
                which we compute as the product of the reciprocals of the weights
                of the two states. */
            virtual double abstractEdgeWeight(ProductGraph::State *a, ProductGraph::State *b) const;

            virtual double abstractEdgeWeightScout(ProductGraph::State *a, ProductGraph::State *b) const;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            // std::vector<base::StateSamplerPtr> samplers;

            std::unordered_map<int , base::StateSamplerPtr> samplers_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief State sampler */
            base::StateSamplerPtr scout_sampler_;

            /** \brief Control sampler */
            ControlSamplerPtr scout_controlSampler_;

            /** \brief Handle to the control::SpaceInformation object */
            const STLSpaceInformation *stlsi_;

            const STLSpaceInformation *scout_stlsi_;

            /** \brief The high level abstaction used to grow the tree structure */
            ProductGraphPtr abstraction_;

            /** \brief The high level abstaction used to grow the tree structure */
            ProductGraphPtr scout_abstraction_;

            /** \brief Used to sample nonempty regions in which to promote expansion. */
            PDF<ProductGraph::State *> availDist_;

            /** \brief Used to sample nonempty regions in which to promote expansion. */
            PDF<int > scout_availDist_;

            /** \brief A random number generator. */
            RNG rng_;

            /** \brief Set of all motions. */
            std::vector<Motion *> motions_;

            std::vector<Motion *> scout_motions_;

            /** \brief Start state in product graph. */
            ProductGraph::State *prodStart_{nullptr};

            /** \brief Start state in product graph. */
            ProductGraph::State *scout_prodStart_{nullptr};

            /** \brief Time to spend exploring each lead. */
            double exploreTime_;
            
            double scout_exploreTime_;

            bool use_scout_{true};

            /** \brief Map of abstraction states to their details. */
            std::unordered_map<ProductGraph::State *, ProductGraphStateInfo> abstractInfo_;

            /** \brief Map of abstraction states to their details. */
            std::unordered_map<int , CosafeStateInfo> scout_abstractInfo_;

            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            std::shared_ptr<NearestNeighbors<Motion *>> scout_nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.0};

            /** \brief The fraction of time the trajectory is picked as the state to expand towards (if such a state is
             * available) */
            double pruningRadius_;

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {  
                // const base::State* lowLevelState_a = stlsi_->getLowLevelState(a->state);
                // std::cout << a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getSigma().trace() << std::endl;
                // std::cout << b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getSigma().trace() << std::endl;
                double dx = a->state->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX() - b->state->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX();
                double dy = a->state->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY() - b->state->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY();
                // std::cout << dx << " " << dy << std::endl;
                Eigen::Matrix2d cov1 = a->state->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance();
                Eigen::Matrix2d cov2 = b->state->as<CompoundState>()->operator[](0)->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance();
                return sqrt(dx*dx+dy*dy) + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace();
                
            }

            // double distanceFunction(const Motion *a, const Motion *b) const
            // {  
            //     // const base::State* lowLevelState_a = stlsi_->getLowLevelState(a->state);
            //     // std::cout << a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getSigma().trace() << std::endl;
            //     // std::cout << b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getSigma().trace() << std::endl;
            //     double dx = a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX() - b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX();
            //     double dy = a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY() - b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY();
                
            //     Eigen::Matrix2d cov1 = a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getCovariance();
            //     Eigen::Matrix2d cov2 = b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getCovariance();
            //     return sqrt(dx*dx+dy*dy) + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace();
            // }

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunctionScout(const Motion *a, const Motion *b) const
            {  
                // const base::State* lowLevelState_a = stlsi_->getLowLevelState(a->state);
                // std::cout << a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getSigma().trace() << std::endl;
                // std::cout << b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getSigma().trace() << std::endl;
                double dx = a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX() - b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getX();
                double dy = a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY() - b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getY();
                
                Eigen::Matrix2d cov1 = a->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getCovariance();
                Eigen::Matrix2d cov2 = b->state->as<CompoundState>()->operator[](0)->as<R2BeliefSpace::StateType>()->getCovariance();
                return sqrt(dx*dx+dy*dy) + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace();
            }

            int num_vertices_;

        private:
            /** \brief Clears this planner's underlying tree of system states. */
            void clearMotions();
        };
    }
}

#endif
