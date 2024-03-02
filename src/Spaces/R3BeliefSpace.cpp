#include "R3BeliefSpace.h"
#include <unsupported/Eigen/MatrixFunctions>

double R3BeliefSpace::StateType::meanNormWeight_  = -1;
double R3BeliefSpace::StateType::covNormWeight_   = -1;
double R3BeliefSpace::StateType::reachDist_   = -1;

bool R3BeliefSpace::StateType::isReached(ompl::base::State *state, bool relaxedConstraint) const
{

    Eigen::Vector3d stateDiff = this->getMatrixData() - state->as<R3BeliefSpace::StateType>()->getMatrixData();
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

ompl::base::State* R3BeliefSpace::allocState(void) const
{

    StateType *rstate = new StateType(sigma_init_);
    rstate->values = new double[dimension_];

    return rstate;
}

void R3BeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setZ(source->as<StateType>()->getZ());
    destination->as<StateType>()->setSigma(source->as<StateType>()->getSigma());
    destination->as<StateType>()->setLambda(source->as<StateType>()->getLambda());
}

void R3BeliefSpace::freeState(State *state) const
{
    RealVectorStateSpace::freeState(state);
}

double R3BeliefSpace::distance(const State* state1, const State *state2) const //wasserstein distance
{

    //returns the wasserstein distance
    // std::cout << "huh" << state1->as<StateType>()->getX() << std::endl;
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();
    double dz = state1->as<StateType>()->getZ() - state2->as<StateType>()->getZ();
    // return pow(dx*dx+dy*dy, 0.5);
    Eigen::Matrix3d cov1 = state1->as<StateType>()->getCovariance();
    Eigen::Matrix3d cov2 = state2->as<StateType>()->getCovariance();
    return sqrt(dx*dx+dy*dy+dz*dz + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace());
}

void R3BeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Yaw]: ";
    std::cout<<"["<<state->as<R3BeliefSpace::StateType>()->getX()<<", "<<state->as<R3BeliefSpace::StateType>()->getY()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<R3BeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"Sigma is" << std::endl;
    std::cout<<state->as<R3BeliefSpace::StateType>()->getSigma()<<std::endl;
    std::cout<<"Lambda is" << std::endl;
    std::cout<<state->as<R3BeliefSpace::StateType>()->getLambda()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}