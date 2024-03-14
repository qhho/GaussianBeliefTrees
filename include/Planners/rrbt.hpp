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

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRBT_
#define OMPL_CONTROL_PLANNERS_RRT_RRBT_

// #include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>

#include "../Spaces/R2BeliefSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;

namespace ompl
{
    namespace control
    {
        /**
           @anchor gRRBT
           @par Short description
           \ref gRRBT "RRT*" (optimal RRT) is an asymptotically-optimal incremental
           sampling-based motion planning algorithm. \ref gRRBT "RRT*" algorithm is
           guaranteed to converge to an optimal solution, while its
           running time is guaranteed to be a constant factor of the
           running time of the \ref gRRT "RRT". The notion of optimality is with
           respect to a specified OptimizationObjective (set in the ProblemDefinition).
           If a solution path's cost is within a user-specified cost-threshold,
           the algorithm terminates before the elapsed time.
           @par External documentation
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research, Vol 30, No 7, 2011.
           https://arxiv.org/abs/1105.1186
        */

        /** \brief Optimal Rapidly-exploring Random Trees */
        class RRBT : public base::Planner
        {
        public:
            RRBT(const SpaceInformationPtr &si);

            ~RRBT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* (or k_rrg = s \times k_rrg*)
             */
            void setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
                calculateRewiringLowerBounds();
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times
             * k_rrg* > k_rrg*) */
            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            /** \brief Option that delays collision checking procedures.
                When it is enabled, all neighbors are sorted by cost. The
                planner then goes through this list, starting with the lowest
                cost, checking for collisions in order to find a parent. The planner
                stops iterating through the list when a collision free parent is found.
                This prevents the planner from collision checking each neighbor, reducing
                computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC() const
            {
                return delayCC_;
            }

            /** \brief Controls whether the tree is pruned during the search. This pruning removes
                a vertex if and only if it \e and all its descendents passes the pruning condition.
                The pruning condition is whether the lower-bounding estimate of a solution
                constrained to pass the the \e vertex is greater than the current solution.
                Considering the descendents of a vertex prevents removing a descendent
                that may actually be capable of later providing a better solution once
                its incoming path passes through a different vertex (e.g., a change in homotopy class). */
            void setTreePruning(bool prune);

            /** \brief Get the state of the pruning option. */
            bool getTreePruning() const
            {
                return useTreePruning_;
            }

            /** \brief Set the fractional change in solution cost necessary for pruning to occur, i.e.,
                prune if the new solution is at least X% better than the old solution.
                (e.g., 0.0 will prune after every new solution, while 1.0 will never prune.) */
            void setPruneThreshold(const double pp)
            {
                pruneThreshold_ = pp;
            }

            /** \brief Get the current prune states percentage threshold parameter. */
            double getPruneThreshold() const
            {
                return pruneThreshold_;
            }

            /** \brief Use the measure of the pruned subproblem instead of the measure of the entire problem domain (if
            such an expression exists and a solution is present).
            Currently the only method to calculate this measure in closed-form is through a informed sampler, so this
            option also requires that. */
            void setPrunedMeasure(bool informedMeasure);

            /** \brief Get the state of using the pruned measure */
            bool getPrunedMeasure() const
            {
                return usePrunedMeasure_;
            }

            /** \brief Use direct sampling of the heuristic for the generation of random samples (e.g., x_rand).
           If a direct sampling method is not defined for the objective, rejection sampling will be used by default. */
            void setInformedSampling(bool informedSampling);

            /** \brief Get the state direct heuristic sampling */
            bool getInformedSampling() const
            {
                return useInformedSampling_;
            }

            /** \brief Controls whether heuristic rejection is used on samples (e.g., x_rand) */
            void setSampleRejection(bool reject);

            /** \brief Get the state of the sample rejection option */
            bool getSampleRejection() const
            {
                return useRejectionSampling_;
            }

            /** \brief Controls whether heuristic rejection is used on new states before connection (e.g., x_new =
             * steer(x_nearest, x_rand)) */
            void setNewStateRejection(const bool reject)
            {
                useNewStateRejection_ = reject;
            }

            /** \brief Get the state of the new-state rejection option */
            bool getNewStateRejection() const
            {
                return useNewStateRejection_;
            }

            /** \brief Controls whether pruning and new-state rejection uses an admissible cost-to-come estimate or not
             */
            void setAdmissibleCostToCome(const bool admissible)
            {
                useAdmissibleCostToCome_ = admissible;
            }

            /** \brief Get the admissibility of the pruning and new-state rejection heuristic */
            bool getAdmissibleCostToCome() const
            {
                return useAdmissibleCostToCome_;
            }

            /** \brief Controls whether samples are returned in ordered by the heuristic. This is accomplished by
             * generating a batch at a time. */
            void setOrderedSampling(bool orderSamples);

            /** \brief Get the state of sample ordering. */
            bool getOrderedSampling() const
            {
                return useOrderedSampling_;
            }

            /** \brief Set the batch size used for sample ordering*/
            void setBatchSize(unsigned int batchSize)
            {
                batchSize_ = batchSize;
            }

            /** \brief Get the batch size used for sample ordering*/
            unsigned int getBatchSize() const
            {
                return batchSize_;
            }

            /** \brief A \e meta parameter to focusing the search to improving the current solution. This is the
            parameter set by CFOREST.
            For RRT*, search focusing consists of pruning the existing search and limiting future search.
            Specifically, this is accomplished by turning on informed sampling, tree pruning and new-state rejection.
            This flag individually sets the options described above.
            */
            void setFocusSearch(const bool focus)
            {
                setInformedSampling(focus);
                setTreePruning(focus);
                setPrunedMeasure(focus);
                setNewStateRejection(focus);
            }

            /** \brief Get the state of search focusing */
            bool getFocusSearch() const
            {
                return getInformedSampling() && getPrunedMeasure() && getTreePruning() && getNewStateRejection();
            }

            /** \brief Use a k-nearest search for rewiring instead of a r-disc search. */
            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            /** \brief Get the state of using a k-nearest search for rewiring. */
            bool getKNearest() const
            {
                return useKNearest_;
            }

            /** \brief Set the number of attempts to make while performing rejection or informed sampling */
            void setNumSamplingAttempts(unsigned int numAttempts)
            {
                numSampleAttempts_ = numAttempts;
            }

            /** \brief Get the number of attempts to make while performing rejection or informed sampling */
            unsigned int getNumSamplingAttempts() const
            {
                return numSampleAttempts_;
            }

            unsigned int numIterations() const
            {
                return iterations_;
            }

            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }


            void setGoal(std::vector<double> goal)
            {
                goal_ = goal;
            }

            std::vector<double> getGoal() const
            {
                return goal_;
            }

            void setScene(std::string scene_id)
            {
                scene_id_ = scene_id;
            }

            std::string getScene() const
            {
                return scene_id_;
            }

            void setQ(double Q)
            {
                Q_ = Q*Q;
            }

            void setR(double R)
            {
                R_ = R*R;
            }

            void setR_bad(double R_bad)
            {
                R_bad_ = R_bad*R_bad;
            }

            void setMeasurementRegion(std::vector<std::vector<double>> measurementRegion)
            {
                measurementRegion_ = measurementRegion;
            }


            Eigen::Matrix2d A_ol_, B_ol_, A_cl_, B_cl_, A_cl_d_, B_cl_d_;

            unsigned int n_obstacles_;
            std::vector<Eigen::Matrix<float, 6, 3> > A_list_;
            std::vector<Eigen::Matrix<float, 6, 1> > B_list_;
            double p_collision_, erf_inv_result_;
            class Motion;

            class Belief
            {
            public:
                Belief(){
                }

                ~Belief() {
                    // std::cout << "deleting" << std::endl;
                    deleted = true;
                    // parent = nullptr;
                    // motion = nullptr;
                    for (auto b:children){
                        b->parent = nullptr;
                        b->deleted = true;
                        // for (auto it = b->motion->beliefs.begin(); it != b->motion->beliefs.end(); ++it)
                        // {
                        //     if (*it == b)
                        //     {
                        //         b->motion->beliefs.erase(it);
                        //         break;
                        //     }
                        // }
                        // std::cout << "ok deleting " << b->x << " " << b->y << std::endl;
                        // delete b;
                        // std::cout << "ok deleted " <<  b->x << " " << b->y << std::endl;
                    }
                    // std::cout << "deleted" << std::endl;
                }

                double x;

                double y;

                Belief *parent{nullptr};
                
                Motion *motion{nullptr};

                base::Cost cost;

                base::Cost incCost;

                bool inGoal{false};

                Eigen::Matrix2d sigma_;
                Eigen::Matrix2d lambda_{Eigen::Matrix2d::Zero()};

                bool deleted = false;

                std::vector<Belief *> children;
            };

            std::vector<Belief *> toBeDeleted;

            /** \brief Representation of a motion */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates
                 * memory for \e state, \e cost, and \e incCost */
                Motion(const SpaceInformation *si) : state(si->allocState()), control_(si->allocControl()), parent(nullptr), inGoal(false)
                {
                }

                ~Motion() {
                    // std::cout << "deleting motion" << std::endl;
                    // deleted = true;
                    for (int i = 0; i < beliefs.size(); ++i){
                        if (!beliefs[i]->deleted){
                            // OMPL_INFORM("Trying to delete belief");
                            // OMPL_INFORM("%f, %d", beliefs[i]->x, beliefs[i]->deleted);
                            delete beliefs[i];
                            // OMPL_INFORM("Deleted belief");
                        }
                    }
                }

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent;

                Control *control_{nullptr};

                /** \brief Set to true if this vertex is in the goal region */
                bool inGoal;

                /** \brief The cost up to this motion */
                base::Cost cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
                 * computations in the updateChildCosts() method) */
                base::Cost incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;

                /** \brief The set of beliefs for the current motion */

                std::vector<Belief* > beliefs;

                bool deleted = false;
            };

        protected:

            std::vector<std::vector< std::pair<Control*, double> > > tm;

            /** \brief Create the samplers */
            void allocSampler();

            /** \brief Generate a sample */
            bool sampleUniform(base::State *statePtr);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost> &costs, const base::OptimizationObjective &opt)
                  : costs_(costs), opt_(opt)
                {
                }
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i], costs_[j]);
                }
                const std::vector<base::Cost> &costs_;
                const base::OptimizationObjective &opt_;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Gets the neighbours of a given motion, using either k-nearest of radius as appropriate. */
            void getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const;

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
                Returns the number of motions pruned. Depends on the parameter set by
               setPruneStatesImprovementThreshold() */
            int pruneTree(const base::Cost &pruneTreeCost);

            /** \brief Computes the solution cost heuristically as the cost to come from start to the motion plus
                 the cost to go from the motion to the goal. If the parameter \e use_admissible_heuristic
                 (\e setAdmissibleCostToCome()) is true, a heuristic estimate of the cost to come is used;
                 otherwise, the current cost to come to the motion is used (which may overestimate the cost
                 through the motion). */
            base::Cost solutionHeuristic(const Motion *motion) const;

            /** \brief Add the children of a vertex to the given list. */
            void addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion);

            /** \brief Check whether the given motion passes the specified cost threshold, meaning it will be \e kept
             * during pruning */
            bool keepCondition(const Motion *motion, const base::Cost &threshold) const;

            /** \brief Calculate the k_RRG* and r_RRG* terms */
            void calculateRewiringLowerBounds();

            bool checkMotion(Motion * nmotion, base::State* state);

            unsigned int mypropagateWhileValid(const Belief* belief, const Control *control,
                                                                  int steps, Belief* result) const;

            unsigned int mypropagateAndCostWhileValid(const Belief* belief, const Control *control,
                                                                  int steps, Belief* result, double& cost) const;

            void mypropagate(const Belief *belief, const control::Control* control, const double duration, Belief *result) const;

            bool myisValid(const Belief *state) const;

            bool inCollision(const Belief *belief, 
                         double X1, double Y1, 
                         double X2, double Y2) const;

            bool inCollision(const Belief *state) const;

            bool HyperplaneCCValidityChecker(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const double &x_pose, const double &y_pose, const double &z_pose, const Eigen::MatrixXf &PX) const;

            bool appendBelief(Motion* motion, Belief* newbelief);

            bool epsilon_dominates(Belief *a, Belief *b) const;

            bool dominates(Belief *a, Belief*b) const;

            double expectedPathLengthmotionCost(const Belief *s1, const Belief *s2) const;

            double distanceGoal(const Belief *st) const;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief An informed sampler */
            base::InformedSamplerPtr infSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief Option to use k-nearest search for rewiring */
            bool useKNearest_{true};

            /** \brief The rewiring factor, s, so that r_rrt = s \times r_rrt* > r_rrt* (or k_rrt = s \times k_rrt* >
             * k_rrt*) */
            double rewireFactor_{1.1};

            /** \brief A constant for k-nearest rewiring calculations */
            double k_rrt_{0u};

            /** \brief A constant for r-disc rewiring calculations */
            double r_rrt_{0.};

            /** \brief Option to delay and reduce collision checking within iterations */
            bool delayCC_{true};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The best goal motion. */
            Motion *bestGoalMotion_{nullptr};

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion *> goalMotions_;

            /** \brief The status of the tree pruning option. */
            bool useTreePruning_{false};

            /** \brief The tree is pruned when the change in solution cost is greater than this fraction. */
            double pruneThreshold_{.05};

            /** \brief Option to use the informed measure */
            bool usePrunedMeasure_{false};

            /** \brief Option to use informed sampling */
            bool useInformedSampling_{false};

            /** \brief The status of the sample rejection parameter. */
            bool useRejectionSampling_{false};

            /** \brief The status of the new-state rejection parameter. */
            bool useNewStateRejection_{false};

            /** \brief The admissibility of the new-state rejection heuristic. */
            bool useAdmissibleCostToCome_{true};

            /** \brief The number of attempts to make at informed sampling */
            unsigned int numSampleAttempts_{100u};

            /** \brief Option to create batches of samples and order them. */
            bool useOrderedSampling_{false};

            /** \brief The size of the batches. */
            unsigned int batchSize_{1u};

            /** \brief Stores the start states as Motions. */
            std::vector<Motion *> startMotions_;

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The cost at which the graph was last pruned */
            base::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The measure of the problem when we pruned it (if this isn't in use, it will be set to
             * si_->getSpaceMeasure())*/
            double prunedMeasure_{0.};

            /** \brief Number of iterations the algorithm performed */
            unsigned int iterations_{0u};

            double stepSize_{0.1};

            double t_crit_{2.477};

            std::vector<double> goal_;

            std::string scene_id_;

            double Q_{0.1};
            double R_{0.1};
            double R_bad_{0.3};

            std::vector<std::vector<double>> measurementRegion_;

            Eigen::Matrix2d I = Eigen::MatrixXd::Identity(2, 2);
            Eigen::Matrix2d H = Eigen::MatrixXd::Identity(2, 2);
            Eigen::Matrix2d F = Eigen::MatrixXd::Identity(2, 2);

            Eigen::MatrixXd Q;

            ///////////////////////////////////////
            // Planner progress property functions
            std::string numIterationsProperty() const
            {
                return std::to_string(numIterations());
            }
            std::string bestCostProperty() const
            {
                return std::to_string(bestCost().value());
            }
        };
    }
}

#endif