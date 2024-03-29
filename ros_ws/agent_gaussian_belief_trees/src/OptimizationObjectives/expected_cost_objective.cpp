/*
 * expected_cost_objective.cpp
 *
 *  Created on: 2024
 *      Author: Qi Heng Ho
 *
 *  Expected Cost objective. Define different expectation cost objectives.
 */

#include "OptimizationObjectives/expected_cost_objective.hpp"



ob::OptimizationObjectivePtr getExpectedPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ExpectedPathLengthObjective(si));
}

ob::OptimizationObjectivePtr getExpectedControlEffortObjective(const ob::SpaceInformationPtr& si, const int control_dimension)
{
    return ob::OptimizationObjectivePtr(new ExpectedControlEffortObjective(si, control_dimension));
}