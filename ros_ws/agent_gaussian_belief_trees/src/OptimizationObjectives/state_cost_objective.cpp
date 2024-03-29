/*
 * state_cost_objective.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: juandhv (Juan David Hernandez Vega, juandhv@eia.udg.edu)
 *
 *  State cost objective. Define different state cost objectives.
 */

#include "OptimizationObjectives/state_cost_objective.hpp"

// ob::OptimizationObjectivePtr getDirVectorsRiskObjective(const ob::SpaceInformationPtr& si)
// {
//     return ob::OptimizationObjectivePtr(new DirVectorsRiskObjective(si));
// }

// ob::OptimizationObjectivePtr getRiskZonesObjective(const ob::SpaceInformationPtr& si, bool motion_cost_interpolation)
// {
//     return ob::OptimizationObjectivePtr(new RiskZonesObjective(si, motion_cost_interpolation));
// }


ob::OptimizationObjectivePtr getEuclideanPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new EuclideanPathLengthObjective(si));
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
	return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ClearanceObjective(si));
}

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 3.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
}