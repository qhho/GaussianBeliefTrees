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
  
//  #include "ompl/extensions/triangle/PropositionalGridDecomposition.h"
//  #include "ompl/extensions/triangle/GridDecomposition.h"
 #include "PropositionalDecomposition.h"
 #include "ompl/control/planners/ltl/World.h"
 #include "ompl/util/RandomNumbers.h"
 #include "ompl/base/State.h"
 #include "ompl/base/StateSampler.h"
 #include "ompl/base/spaces/RealVectorBounds.h"
 #include <ostream>
 #include <set>
 #include <vector>
  
 namespace ob = ompl::base;
 namespace oc = ompl::control;
  
 namespace
 {
     /* PropositionalGridDecomposition creates a WrapperDecomposition under-the-hood
        (which represents a triangulation over the given bounds that uses
        PropositionalGridDecomposition's user-defined project and sample methods)
        and hands it upward to the PropositionalDecomposition superclass constructor. */
     class WrapperDecomposition : public oc::GridDecomposition
     {
     public:
         using Polygon = GridDecomposition::Polygon;
         using Vertex = GridDecomposition::Vertex;
         WrapperDecomposition(const oc::Decomposition *decomp, const ob::RealVectorBounds &bounds,
                              const std::vector<Polygon> &holes, const std::vector<Polygon> &props);
         ~WrapperDecomposition() override = default;
         void project(const ob::State *s, std::vector<double> &coord) const override;
         void sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const override;
         void sampleFullState(const ob::StateSamplerPtr &sampler, const std::vector<double> &coord,
                              ob::State *s) const override;
  
     protected:
         const oc::Decomposition *decomp_;
     };
 }
  
 oc::PropositionalGridDecomposition::PropositionalGridDecomposition(const ob::RealVectorBounds &bounds,
                                                                                const std::vector<Polygon> &holes,
                                                                                const std::vector<Polygon> &props)
   : PropositionalDecomposition(std::make_shared<WrapperDecomposition>(this, bounds, holes, props))
   , triDecomp_(static_cast<GridDecomposition *>(decomp_.get()))
 {
 }
  
 int oc::PropositionalGridDecomposition::getNumProps() const
 {
     return triDecomp_->getNumRegionsOfInterest();
 }
  
 oc::World oc::PropositionalGridDecomposition::worldAtRegion(int triID)
 {
     int numProps = getNumProps();
     World world(numProps);
     for (int p = 0; p < numProps; ++p)
         world[p] = false;
     if (triID == -1)
         return world;
     int prop = triDecomp_->getRegionOfInterestAt(triID);
     if (prop >= 0)
         world[prop] = true;
     return world;
 }
  
 void oc::PropositionalGridDecomposition::setup()
 {
     triDecomp_->setup();
 }
  
 void oc::PropositionalGridDecomposition::addHole(const Polygon &hole)
 {
     triDecomp_->addHole(hole);
 }
  
 void oc::PropositionalGridDecomposition::addProposition(const Polygon &prop)
 {
     triDecomp_->addRegionOfInterest(prop);
 }
  
 const std::vector<oc::PropositionalGridDecomposition::Polygon> &
 oc::PropositionalGridDecomposition::getHoles() const
 {
     return triDecomp_->getHoles();
 }
  
 const std::vector<oc::PropositionalGridDecomposition::Polygon> &
 oc::PropositionalGridDecomposition::getPropositions() const
 {
     return triDecomp_->getAreasOfInterest();
 }
  
 void oc::PropositionalGridDecomposition::print(std::ostream &out) const
 {
     triDecomp_->print(out);
 }
  
 namespace
 {
     WrapperDecomposition::WrapperDecomposition(const oc::Decomposition *decomp, const ob::RealVectorBounds &bounds,
                                                const std::vector<Polygon> &holes, const std::vector<Polygon> &props)
       : oc::GridDecomposition(bounds, holes, props), decomp_(decomp)
     {
     }
  
     void WrapperDecomposition::project(const ob::State *s, std::vector<double> &coord) const
     {
         decomp_->project(s, coord);
     }
  
     void WrapperDecomposition::sampleFromRegion(int rid, ompl::RNG &rng, std::vector<double> &coord) const
     {
         decomp_->sampleFromRegion(rid, rng, coord);
     }
  
     void WrapperDecomposition::sampleFullState(const ob::StateSamplerPtr &sampler, const std::vector<double> &coord,
                                                ob::State *s) const
     {
         decomp_->sampleFullState(sampler, coord, s);
     }
 }