//OMPL
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

double dotprod(const oc::Control *u, const int dimension) {
    double result = 0.0;

    for (int i = 0; i < dimension; i++) {
        result += u->as<oc::RealVectorControlSpace::ControlType>()->values[i] * u->as<oc::RealVectorControlSpace::ControlType>()->values[i];
    }
    return result;
}