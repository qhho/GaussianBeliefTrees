/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
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
*   * Neither the name of the Texas A&M University nor the names of its
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

/* Authors: Qi Heng Ho (adapted from Saurav Agarwal) */

#ifndef R3BELIEF_SPACE_H_
#define R3BELIEF_SPACE_H_

// OMPL includes
#include "ompl/base/spaces/RealVectorStateSpace.h"

//other includes
#include <boost/math/constants/constants.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

using namespace ompl::base;
class R3BeliefSpace : public ompl::base::RealVectorStateSpace
{

    public:

        /** \brief A belief in R(3): (x, y, z, covariance) */
        class StateType : public RealVectorStateSpace::StateType
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            StateType(double sigma_init = 5.0) : RealVectorStateSpace::StateType()
            {
              
              sigma_ = sigma_init*Eigen::MatrixXd::Identity(3,3);
              this->setLambda(Eigen::Matrix3d::Zero());
            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return this->values[0];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return this->values[1];
            }

            /** \brief Get the Y component of the state */
            double getZ(void) const
            {
                return this->values[2];
            }

            const Eigen::Vector3d getXYZ(void) const
            {
                const Eigen::Vector3d stateVec(getX(), getY(), getZ());
                return stateVec;
            }

            Eigen::Matrix3d getSigma(void) const
            {    
                return sigma_;
            }

            Eigen::Matrix3d getLambda(void) const
            {
                return lambda_;
            }

            Eigen::Matrix3d getCovariance(void) const
            {
                return sigma_ + lambda_;
            }

            /** \brief Set the X~ component of the state */
            void setX(double x)
            {
                this->values[0] = x;
            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                this->values[1] = y;
            }

             /** \brief Set the Z component of the state */
            void setZ(double z)
            {
                this->values[2] = z;
            }

            /** \brief Set the X and Y components of the state */
            void setXYZ(double x, double y, double z)
            {
                setX(x);
                setY(y);
                setZ(z);
            }    

            void setMatrixData(const Eigen::Vector3d &x)
            {
                setX(x[0]);
                setY(x[1]);
                setZ(x[2]);
            }

            void setSigmaX(double val){
                sigma_(0,0) = val;
            }

            void setSigmaY(double val){
                sigma_(1,1) = val;
            }

            void setSigmaZ(double val){
                sigma_(2,2) = val;
            }

            void setSigma(Eigen::Matrix3d cov){
                sigma_ = cov;
            }

            void setSigma(double val){
                sigma_ = val*Eigen::MatrixXd::Identity(3,3);
            }

            void setLambda(Eigen::Matrix3d cov){
                lambda_ = cov;
            }

            Eigen::Vector3d getMatrixData(void) const
            {
                Eigen::Vector3d stateVec(getX(), getY(), getZ());
                return stateVec;
            }

            void setCost(double cost){
                cost_ = cost;
            }

            double getCost(void) const{ 
                return cost_;
            }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;

            static double meanNormWeight_, covNormWeight_, reachDist_;

            int DISTANCE_FUNCTION_TYPE_;
            
        private:
              Eigen::Matrix3d sigma_;
              Eigen::Matrix3d lambda_;
              int dimensions_ = 3;
              double cost_;
              double sigma_init_ = 5.0;

        };

        R3BeliefSpace(double sigma_init = 5.0) : RealVectorStateSpace(3)
        {
            setName("R3_BELIEF" + getName());
            type_ = STATE_SPACE_REAL_VECTOR;
            sigma_init_ = sigma_init;
        }

        virtual ~R3BeliefSpace(void)
        {
        }

        virtual State* allocState(void) const override;

        virtual void copyState(State *destination,const State *source) const override;

        virtual void freeState(State *state) const override;

        //virtual void registerProjections(void);
        virtual double distance(const State* state1, const State *state2) const override;

        // virtual double distance(State* state1, State *state2)override;

        // gets the relative vector between "from" and "to"
        // equivalent to result = vectorA-vectorB
        void getRelativeState(const State *from, const State *to, State *state);

        void printBeliefState(const State *state);
        
        protected:
            double sigma_init_;

};
#endif