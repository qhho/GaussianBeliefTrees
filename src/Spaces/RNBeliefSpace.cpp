#include "Spaces/RNBeliefSpace.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <ompl/util/Exception.h>
#include <iostream>

// Initialize static members
double RNBeliefSpace::StateType::meanNormWeight_ = -1;
double RNBeliefSpace::StateType::covNormWeight_ = -1;
double RNBeliefSpace::StateType::reachDist_ = -1;

RNBeliefSpace::StateType::StateType(const Eigen::MatrixXd &sigma_init) 
    : sigma_(sigma_init),
      lambda_(Eigen::MatrixXd::Zero(sigma_init.rows(), sigma_init.cols()))
{
}

double RNBeliefSpace::StateType::getComponent(unsigned int index) const
{
    return values[index];
}

void RNBeliefSpace::StateType::setComponent(unsigned int index, double value)
{
    values[index] = value;
}

double RNBeliefSpace::StateType::getX() const
{
    return values[0];
}

double RNBeliefSpace::StateType::getY() const
{
    return (sigma_.rows() > 1) ? values[1] : 0.0;
}

double RNBeliefSpace::StateType::getZ() const
{
    return (sigma_.rows() > 2) ? values[2] : 0.0;
}

void RNBeliefSpace::StateType::setX(double x)
{
    values[0] = x;
}

void RNBeliefSpace::StateType::setY(double y)
{
    if (sigma_.rows() > 1) values[1] = y;
}

void RNBeliefSpace::StateType::setZ(double z)
{
    if (sigma_.rows() > 2) values[2] = z;
}

void RNBeliefSpace::StateType::SetSigmaRandom(double max)
{
    unsigned int dim = sigma_.rows();
    for (unsigned int i = 0; i < dim; i++) {
        sigma_(i,i) = rng_.uniform01()*max;
    }
}

Eigen::VectorXd RNBeliefSpace::StateType::getMatrixData() const
{
    unsigned int dim = sigma_.rows();
    Eigen::VectorXd mean(dim);
    for (unsigned int i = 0; i < dim; i++) {
        mean(i) = values[i];
    }
    return mean;
}

const Eigen::MatrixXd& RNBeliefSpace::StateType::getSigma() const
{
    return sigma_;
}

const Eigen::MatrixXd& RNBeliefSpace::StateType::getLambda() const
{
    return lambda_;
}

const Eigen::MatrixXd& RNBeliefSpace::StateType::getCovariance() const
{
    return getSigma();
}

void RNBeliefSpace::StateType::setSigma(double val)
{
    sigma_ = val*Eigen::MatrixXd::Identity(sigma_.rows(),sigma_.cols());
}


void RNBeliefSpace::StateType::setSigma(const Eigen::MatrixXd &sigma)
{
    if (sigma.rows() != sigma_.rows() || sigma.cols() != sigma_.cols()) {
        throw ompl::Exception("Sigma matrix dimension mismatch");
    }
    sigma_ = sigma;
}

void RNBeliefSpace::StateType::setSigmaX(double val)
{
    sigma_(0,0) = val;
}

void RNBeliefSpace::StateType::setSigmaY(double val)
{
    sigma_(1,1) = val;
}

void RNBeliefSpace::StateType::setLambda(const Eigen::MatrixXd &lambda)
{
    if (lambda.rows() != lambda_.rows() || lambda.cols() != lambda_.cols()) {
        throw ompl::Exception("Lambda matrix dimension mismatch");
    }
    lambda_ = lambda;
}

bool RNBeliefSpace::StateType::isReached(ob::State *state, bool relaxedConstraint) const
{
    Eigen::VectorXd stateDiff = this->getMatrixData() - state->as<RNBeliefSpace::StateType>()->getMatrixData();
    double meanNorm = stateDiff.norm();
    double reachConstraint = reachDist_;
    
    if (relaxedConstraint) {
        reachConstraint *= 4;
    }
    
    if (meanNorm <= reachConstraint) {
        return true;
    }
    
    return false;
}

RNBeliefSpace::RNBeliefSpace(unsigned int dimension, const Eigen::MatrixXd &sigma_init)
    : ob::RealVectorStateSpace(dimension), dimension_(dimension), sigma_init_(sigma_init), wasserstein_(1)
{
    // Validate covariance matrix dimensions
    if (sigma_init.rows() != dimension || sigma_init.cols() != dimension) {
        throw ompl::Exception("Initial covariance matrix dimensions must match state dimension");
    }
    
    setName("RNBeliefSpace");
}

RNBeliefSpace::RNBeliefSpace(unsigned int dimension, bool wasserstein, const Eigen::MatrixXd &sigma_init)
    : ob::RealVectorStateSpace(dimension), dimension_(dimension), sigma_init_(sigma_init), wasserstein_(wasserstein)
{
    // Validate covariance matrix dimensions
    if (sigma_init.rows() != dimension || sigma_init.cols() != dimension) {
        throw ompl::Exception("Initial covariance matrix dimensions must match state dimension");
    }
    
    setName("RNBeliefSpace");
}

void RNBeliefSpace::setDistanceParams(double meanNormWeight, double covNormWeight, double reachDist)
{
    StateType::meanNormWeight_ = meanNormWeight;
    StateType::covNormWeight_ = covNormWeight;
    StateType::reachDist_ = reachDist;
}

ob::State* RNBeliefSpace::allocState(void) const
{
    StateType *rstate = new StateType(sigma_init_);
    rstate->values = new double[dimension_];
    return rstate;
}

void RNBeliefSpace::freeState(ob::State *state) const
{
    RealVectorStateSpace::freeState(state);
}

void RNBeliefSpace::copyState(ob::State *destination, const ob::State *source) const
{
    // Copy position values
    for (unsigned int i = 0; i < dimension_; i++) {
        destination->as<StateType>()->setComponent(i, source->as<StateType>()->getComponent(i));
    }
    
    // Copy matrices
    destination->as<StateType>()->setSigma(source->as<StateType>()->getSigma());
    destination->as<StateType>()->setLambda(source->as<StateType>()->getLambda());
}

double RNBeliefSpace::distance(const ob::State *state1, const ob::State *state2) const
{
    // Wasserstein distance implementation
    const StateType *s1 = state1->as<StateType>();
    const StateType *s2 = state2->as<StateType>();
    
    // Calculate mean difference
    double dx = 0;
    for (unsigned int i = 0; i < dimension_; i++) {
        double diff = s1->getComponent(i) - s2->getComponent(i);
        dx += diff * diff;
    }

    if (!wasserstein_)
    {
        return std::sqrt(dx);
    }
    
    // Get covariance matrices
    Eigen::MatrixXd cov1 = s1->getCovariance();
    Eigen::MatrixXd cov2 = s2->getCovariance();

    // std::cout << " --- COVARIANCE--- " << std::endl;
    // std::cout << cov1 << std::endl;
    // std::cout << cov2 << std::endl;

    // std::cout << " ----------- " << std::endl;
    
    // Calculate covariance component of Wasserstein distance
    try {
        // Numerical safeguard
        const double epsilon = 1e-9;
        Eigen::MatrixXd cov2_sqrt;
        
        try {
            cov2_sqrt = cov2.sqrt();
        } catch (...) {
            cov2_sqrt = (cov2 + epsilon * Eigen::MatrixXd::Identity(dimension_, dimension_)).sqrt();
        }
        
        Eigen::MatrixXd inner;
        try {
            
            // std::cout << cov2_sqrt*cov1*cov2_sqrt << std::endl;
            // std::cout <<"computing inner " << std::endl;


            // Eigen::LLT<Eigen::MatrixXd> lltOfA(cov2_sqrt*cov1*cov2_sqrt); // compute the Cholesky decomposition of A
            //     if(lltOfA.info() == Eigen::NumericalIssue)
            //     {
            //         throw std::runtime_error("Possibly non semi-positive definitie matrix!");
            //     }
            inner = (cov1 * cov2 + epsilon * Eigen::MatrixXd::Identity(dimension_, dimension_)).sqrt();
            // std::cout << "computed inner" << std::endl;
        } catch (...) {
            Eigen::MatrixXd temp = cov1 * cov2;
            inner = (temp + epsilon * Eigen::MatrixXd::Identity(dimension_, dimension_)).sqrt();
        }
        
        double covDist = (cov1 + cov2 - 2 * inner).trace();
        
        // If distance is very small, return 0
        if (std::sqrt(dx + covDist) < 1e-5) {
            return 0.0;
        }
        
        // Return weighted combination
        // std::cout << "done ---" << std::endl;
        return std::sqrt(dx) + covDist;
    } catch (...) {
        
        // Fallback to just mean distance if any numerical issues
        return std::sqrt(dx);
    }
}

void RNBeliefSpace::printBeliefState(const ob::State *state)
{
    std::cout << "----Printing BeliefState----" << std::endl;
    
    // Print mean
    std::cout << "State [";
    for (unsigned int i = 0; i < dimension_; i++) {
        std::cout << state->as<StateType>()->getComponent(i);
        if (i < dimension_ - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // Print covariance
    std::cout << "Covariance is" << std::endl;
    std::cout << state->as<StateType>()->getCovariance() << std::endl;
    
    // Print sigma
    std::cout << "Sigma is" << std::endl;
    std::cout << state->as<StateType>()->getSigma() << std::endl;
    
    // Print lambda
    std::cout << "Lambda is" << std::endl;
    std::cout << state->as<StateType>()->getLambda() << std::endl;
    
    std::cout << "------End BeliefState-------" << std::endl;
}