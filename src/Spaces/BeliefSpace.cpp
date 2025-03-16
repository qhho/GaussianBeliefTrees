/**
 * @file BeliefStateSpace.cpp
 * @brief Implementation of the belief state space for linear systems in OMPL
 */

 #include "BeliefStateSpace.h"
 #include <ompl/base/StateSpace.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/base/ProjectionEvaluator.h>
 
 /**
  * @class BeliefMeanProjection
  * @brief Projects a belief state to just its mean components
  */
 class BeliefMeanProjection : public ob::ProjectionEvaluator {
 public:
     BeliefMeanProjection(const BeliefStateSpace *space)
         : ob::ProjectionEvaluator(space),
           space_(space) {
         unsigned int dim = space_->getStateDimension();
         projDim_ = dim;  // Project to just the mean state
     }
     
     unsigned int getDimension() const override {
         return projDim_;
     }
     
     void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override {
         for (unsigned int i = 0; i < projDim_; ++i) {
             projection(i) = space_->getMeanComponent(state, i);
         }
     }
     
 private:
     const BeliefStateSpace *space_;
     unsigned int projDim_;
 };
 
 BeliefStateSpace::BeliefStateSpace(unsigned int dim)
     : ob::RealVectorStateSpace(dim + 2 * dim * dim),  // Mean + Cov + Info
       stateDim_(dim) {
     
     // Calculate total dimension
     totalDim_ = stateDim_ + 2 * stateDim_ * stateDim_;
     
     // Setting bounds for the mean (default [-1, 1] for each dimension)
     // Can be modified later with setBounds()
     ob::RealVectorBounds bounds(totalDim_);
     
     // Set bounds for the mean components
     for (unsigned int i = 0; i < stateDim_; ++i) {
         bounds.setLow(i, -1.0);
         bounds.setHigh(i, 1.0);
     }
     
     // Set bounds for covariance and information matrices
     // These are large to accommodate reasonable matrices
     for (unsigned int i = stateDim_; i < totalDim_; ++i) {
         bounds.setLow(i, -1000.0);
         bounds.setHigh(i, 1000.0);
     }
     
     setBounds(bounds);
     
     // Set the dimension name
     setName("BeliefStateSpace" + std::to_string(dim));
 }
 
 unsigned int BeliefStateSpace::getStateDimension() const {
     return stateDim_;
 }
 
 void BeliefStateSpace::setMean(ob::State *state, const Eigen::VectorXd &mean) const {
     if (mean.size() != stateDim_) {
         throw ompl::Exception("Mean vector dimension mismatch");
     }
     
     auto *realVecState = state->as<ob::RealVectorStateSpace::StateType>();
     
     for (unsigned int i = 0; i < stateDim_; ++i) {
         realVecState->values[i] = mean(i);
     }
 }
 
 Eigen::VectorXd BeliefStateSpace::getMean(const ob::State *state) const {
     Eigen::VectorXd mean(stateDim_);
     const auto *realVecState = state->as<ob::RealVectorStateSpace::StateType>();
     
     for (unsigned int i = 0; i < stateDim_; ++i) {
         mean(i) = realVecState->values[i];
     }
     
     return mean;
 }
 
 void BeliefStateSpace::setCovariance(ob::State *state, const Eigen::MatrixXd &cov) const {
     if (cov.rows() != stateDim_ || cov.cols() != stateDim_) {
         throw ompl::Exception("Covariance matrix dimension mismatch");
     }
     
     auto *realVecState = state->as<ob::RealVectorStateSpace::StateType>();
     
     unsigned int idx = stateDim_;
     for (unsigned int i = 0; i < stateDim_; ++i) {
         for (unsigned int j = 0; j < stateDim_; ++j) {
             realVecState->values[idx++] = cov(i, j);
         }
     }
 }
 
 Eigen::MatrixXd BeliefStateSpace