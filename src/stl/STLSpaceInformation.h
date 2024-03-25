/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

#ifndef OMPL_CONTROL_PLANNERS_STL_STLSPACEINFORMATION_
#define OMPL_CONTROL_PLANNERS_STL_STLSPACEINFORMATION_

#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../Spaces/R2BeliefSpace.h"
#include "ompl/control/SpaceInformation.h"
#include "STLProductGraph.h"

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::STLSpaceInformation */
        OMPL_CLASS_FORWARD(STLSpaceInformation);
        /// @endcond

        /** \class ompl::control::STLSpaceInformationPtr
            \brief A shared pointer wrapper for ompl::control::STLSpaceInformation */
        class STLSpaceInformation : public SpaceInformation
        {
        public:
            STLSpaceInformation(const SpaceInformationPtr &si, const ProductGraphPtr &prod);

            ~STLSpaceInformation() override = default;

            void setup() override;

            const ProductGraphPtr &getProductGraph() const
            {
                return prod_;
            }

            const SpaceInformationPtr &getLowSpace()
            {
                return lowSpace_;
            }

            void getFullState(const base::State *low, base::State *full);

            base::State *getLowLevelState(base::State *s);
            const base::State *getLowLevelState(const base::State *s);

            base::State *getTime(base::State *s);
            const base::State *getTime(const base::State *s);

            ProductGraph::State *getProdGraphState(const base::State *s) const;

        protected:
            enum SpaceIndex
            {
                LOW_LEVEL = 0,
                REGION = 1,
                COSAFE = 2,
                SAFE = 3,
                TIME = 4
            };
            // Creates a new state propagator which uses the user-defined propagator
            // defined in low state space, followed by automaton-defined transition rules
            // for the co-safe and safe automata in prod.
            void extendPropagator(const SpaceInformationPtr &oldsi);

            // Creates a new state validity checker which uses the user-defined validity checker
            // defined in low state space and also checks for valid states in the automata in prod.
            void extendValidityChecker(const SpaceInformationPtr &oldsi);

            ProductGraphPtr prod_;
            SpaceInformationPtr lowSpace_;
        };
    }
}

#endif
