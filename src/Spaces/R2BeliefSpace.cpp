#include "Spaces/R2BeliefSpace.h"
#include <unsupported/Eigen/MatrixFunctions>

double R2BeliefSpace::StateType::meanNormWeight_  = -1;
double R2BeliefSpace::StateType::covNormWeight_   = -1;
double R2BeliefSpace::StateType::reachDist_   = -1;

bool R2BeliefSpace::StateType::isReached(ompl::base::State *state, bool relaxedConstraint) const
{

    Eigen::Vector2d stateDiff = this->getMatrixData() - state->as<R2BeliefSpace::StateType>()->getMatrixData();
    double meanNorm = stateDiff.norm();

    double reachConstraint  = reachDist_;

    if(relaxedConstraint)
        reachConstraint *= 4;

    if(meanNorm <= reachConstraint)
    {
        return true;
    }

    return false;
    
}

ompl::base::State* R2BeliefSpace::allocState(void) const
{

    StateType *rstate = new StateType(sigma_init_);
    rstate->values = new double[dimension_];

    return rstate;
}

void R2BeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setSigma(source->as<StateType>()->getSigma());
    destination->as<StateType>()->setLambda(source->as<StateType>()->getLambda());
}

void R2BeliefSpace::freeState(State *state) const
{
    RealVectorStateSpace::freeState(state);
}

double R2BeliefSpace::distance(const State* state1, const State *state2) const //wasserstein distance
{

    //returns the wasserstein distance
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();
    Eigen::Matrix2d cov1 = state1->as<StateType>()->getCovariance();
    Eigen::Matrix2d cov2 = state2->as<StateType>()->getCovariance();
    return pow(dx*dx+dy*dy, 0.5) + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace();
}

void R2BeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Yaw]: ";
    std::cout<<"["<<state->as<R2BeliefSpace::StateType>()->getX()<<", "<<state->as<R2BeliefSpace::StateType>()->getY()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<R2BeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"Sigma is" << std::endl;
    std::cout<<state->as<R2BeliefSpace::StateType>()->getSigma()<<std::endl;
    std::cout<<"Lambda is" << std::endl;
    std::cout<<state->as<R2BeliefSpace::StateType>()->getLambda()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}