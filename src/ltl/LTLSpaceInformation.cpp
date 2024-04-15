#include <utility>

#include "LTLSpaceInformation.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/StatePropagator.h"
#include "STLProductGraph.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    // Helper method to take a robot state space and product graph and return
    // the hybrid state space representing their product.
    static ob::StateSpacePtr extendStateSpace(const ob::StateSpacePtr &lowSpace, const oc::ProductGraphPtr &prod);
}

oc::STLSpaceInformation::STLSpaceInformation(const oc::SpaceInformationPtr &si, const oc::ProductGraphPtr &prod)
  : oc::SpaceInformation(extendStateSpace(si->getStateSpace(), prod), si->getControlSpace()), prod_(prod), lowSpace_(si)
{
    //\todo: Technically there's a bug here, as we've assigning STLSpaceInformation's
    //      control space to be si->getControlSpace(), which internally holds a pointer
    //      to si->getStateSpace() instead of this->getStateSpace(). In practice, this
    //      is fine for now, since control space never actually uses its internal state
    //      space pointer.
    extendPropagator(si);
    extendValidityChecker(si);
}

void oc::STLSpaceInformation::setup()
{
    // Set up the low space, then match our parameters to it.
    if (!lowSpace_->isSetup())
        lowSpace_->setup();
    // We never actually use the below parameters in STLSpaceInformation while planning.
    // All integrating is done in lowSpace. However, we will need these parameters when
    // printing the path - PathControl::print() will convert path steps using these
    // parameters.
    setMinMaxControlDuration(lowSpace_->getMinControlDuration(), lowSpace_->getMaxControlDuration());
    setPropagationStepSize(lowSpace_->getPropagationStepSize());
    setup_ = true;
}

void oc::STLSpaceInformation::getFullState(const ob::State *low, ob::State *full)
{
    const ProductGraph::State *high = prod_->getState(low);
    ob::CompoundState &cs = *full->as<ob::CompoundState>();
    stateSpace_->as<ob::CompoundStateSpace>()->getSubspace(LOW_LEVEL)->copyState(cs[LOW_LEVEL], low);
    using DiscreteState = ob::DiscreteStateSpace::StateType;
    cs[REGION]->as<DiscreteState>()->value = high->getDecompRegion();
    cs[COSAFE]->as<DiscreteState>()->value = high->getCosafeState();
    cs[SAFE]->as<DiscreteState>()->value = high->getSafeState();
}

ob::State *oc::STLSpaceInformation::getLowLevelState(ob::State *s)
{
    return const_cast<ob::State *>(getLowLevelState(const_cast<const ob::State *>(s)));
}

const ob::State *oc::STLSpaceInformation::getLowLevelState(const ob::State *s)
{
    return s->as<ob::CompoundState>()->operator[](LOW_LEVEL);
}

ob::State *oc::STLSpaceInformation::getTime(ob::State *s)
{
    return const_cast<ob::State *>(getTime(const_cast<const ob::State *>(s)));
}

const ob::State *oc::STLSpaceInformation::getTime(const ob::State *s)
{
    return s->as<ob::CompoundState>()->operator[](TIME);
}

oc::ProductGraph::State *oc::STLSpaceInformation::getProdGraphState(const ob::State *s) const
{
    const ob::CompoundState &cs = *s->as<ob::CompoundState>();
    using DiscreteState = ob::DiscreteStateSpace::StateType;
    return prod_->getState(cs[REGION]->as<DiscreteState>()->value, cs[COSAFE]->as<DiscreteState>()->value,
                           cs[SAFE]->as<DiscreteState>()->value);
}

void oc::STLSpaceInformation::extendPropagator(const oc::SpaceInformationPtr &oldsi) //need to add determinizing of states
{
    class STLStatePropagator : public oc::StatePropagator
    {
    public:
        STLStatePropagator(oc::STLSpaceInformation *ltlsi, oc::ProductGraphPtr prod, oc::StatePropagatorPtr lowProp)
          : oc::StatePropagator(ltlsi), prod_(std::move(prod)), lowProp_(std::move(lowProp)), stlsi_(ltlsi)
        {
        }
        ~STLStatePropagator() override = default;

        void propagate(const ob::State *state, const oc::Control *control, const double duration,
                       ob::State *result) const override
        {
            const ob::State *lowLevelPrev = stlsi_->getLowLevelState(state);
            ob::State *lowLevelResult = stlsi_->getLowLevelState(result);
            lowProp_->propagate(lowLevelPrev, control, duration, lowLevelResult);

            const oc::ProductGraph::State *prevHigh = stlsi_->getProdGraphState(state);
            std::vector<oc::ProductGraph::State*> nextHighs = prod_->getStates(prevHigh, lowLevelResult);
            result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(TIME)->values[0] = 
            stlsi_->getTime(state)->as<ob::RealVectorStateSpace::StateType>()->values[0] + duration;
            double currentTime = result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(TIME)->values[0];
            const oc::ProductGraph::State *nextHigh = NULL;
            if (nextHighs.size() == 0){
                std::cout << "DOOMED";
            }
            else if (nextHighs.size() == 1){
                nextHigh = nextHighs[0];
            }
            result->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(REGION)->value =
                nextHigh->getDecompRegion();
            result->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(COSAFE)->value =
                nextHigh->getCosafeState();
            result->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(SAFE)->value =
                nextHigh->getSafeState();
        }

        bool canPropagateBackward() const override
        {
            return lowProp_->canPropagateBackward();
        }

    private:
        const oc::ProductGraphPtr prod_;
        const oc::StatePropagatorPtr lowProp_;
        oc::STLSpaceInformation *stlsi_;
    };

    // Some compilers have trouble with STLStatePropagator being hidden in this function,
    // and so we explicitly cast it to its base type.
    setStatePropagator(std::make_shared<STLStatePropagator>(this, prod_, oldsi->getStatePropagator()));
}

void oc::STLSpaceInformation::extendValidityChecker(const oc::SpaceInformationPtr &oldsi)
{
    class STLStateValidityChecker : public ob::StateValidityChecker
    {
    public:
        STLStateValidityChecker(oc::STLSpaceInformation *ltlsi, oc::ProductGraphPtr prod,
                                ob::StateValidityCheckerPtr lowChecker)
          : ob::StateValidityChecker(ltlsi), prod_(std::move(prod)), lowChecker_(std::move(lowChecker)), stlsi_(ltlsi)
        {
        }
        ~STLStateValidityChecker() override = default;
        bool isValid(const ob::State *s) const override
        {            
            // double lowbound = 0.0;
            // double upbound = 100.0;
            // double currentTime = stlsi_->getTime(s)->as<ob::RealVectorStateSpace::StateType>()->values[0];
            return stlsi_->getProdGraphState(s)->isValid() && lowChecker_->isValid(stlsi_->getLowLevelState(s));
        }
    private:
        const oc::ProductGraphPtr prod_;
        const ob::StateValidityCheckerPtr lowChecker_;
        oc::STLSpaceInformation *stlsi_;
    };

    // Some compilers have trouble with STLStateValidityChecker being hidden in this function,
    // and so we explicitly cast it to its base type.
    setStateValidityChecker(std::make_shared<STLStateValidityChecker>(this, prod_, oldsi->getStateValidityChecker()));
}

namespace
{
    ob::StateSpacePtr extendStateSpace(const ob::StateSpacePtr &lowSpace, const oc::ProductGraphPtr &prod)
    {
        const oc::AutomatonPtr cosafe(prod->getCosafetyAutom());
        const oc::AutomatonPtr safe(prod->getSafetyAutom());
        auto regionSpace(std::make_shared<ob::DiscreteStateSpace>(0, prod->getDecomp()->getNumRegions() - 1));
        auto cosafeSpace(std::make_shared<ob::DiscreteStateSpace>(0, cosafe->numStates() - 1));
        auto safeSpace(std::make_shared<ob::DiscreteStateSpace>(0, safe->numStates() - 1));
        auto compound(std::make_shared<ob::CompoundStateSpace>());
        compound->addSubspace(lowSpace, 1.);
        compound->addSubspace(regionSpace, 0.);
        compound->addSubspace(cosafeSpace, 0.);
        compound->addSubspace(safeSpace, 0.);
        compound->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0);
        compound->lock();

        compound->as<ob::RealVectorStateSpace>(4)->setBounds(0.0, 1000.0);

        return compound;
    }
}
