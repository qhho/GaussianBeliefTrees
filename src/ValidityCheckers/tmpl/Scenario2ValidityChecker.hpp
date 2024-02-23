/*
 * state_validity_checker_hyperplane_SE2_V.hpp
 *
 *  Created on: Apr 22, 2015
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *      
 *  State checker. Check is a given configuration (SE2 state) is collision-free.
 *  The workspace is represented by convex obstacles (hyperplanes).
 */

#ifndef OMPL_CONTRIB_STATE_VALIDITY_CHECKER_SCENARIO2
#define OMPL_CONTRIB_STATE_VALIDITY_CHECKER_SCENARIO2


//Standard libraries
#include <cstdlib>
#include <cmath>
#include <string>

//OMPL
#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/debug/Profiler.h>

//Boost
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

#include <iostream>


//headers

#include "../Spaces/R2BeliefSpace.h"

//OMPL namespaces
namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  HyperplaneStateValidityCheckerSE2V class.
/*!
  Octomap State Validity checker.
  Extension of an abstract class used to implement the state validity checker over an octomap using FCL.
*/
class Scenario2ValidityChecker : public ob::StateValidityChecker {

public:
    //! Scenario2ValidityChecker constructor.
    Scenario2ValidityChecker(const ob::SpaceInformationPtr &si);

    //! Scenario2ValidityChecker destructor.
    /*!
     * Destroy
     */
    ~Scenario2ValidityChecker();

    // bool inCollisionWithObstacle(const ob::State *state, std::vector<double > obstacle);

    bool inCollision(const ob::State *state, 
                         double X1, double Y1, 
                         double X2, double Y2) const;
    bool inCollision(const ob::State *state) const;

    //! State validator.
    /*!
     * Function that verifies if the given state is valid (i.e. is free of collision) using FCL
     */
    virtual bool isValid(const ob::State *state) const;

private:

    //Eigen for matrix and vector
    Eigen::MatrixXd A1_, A2_, A3_;
    Eigen::MatrixXd B1_, B2_, B3_;
};

#endif
