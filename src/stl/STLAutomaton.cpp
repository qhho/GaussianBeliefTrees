/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Matt Maly, Keliang He */

#include "STLAutomaton.hpp"
#include "ompl/control/planners/ltl/World.h"
#if OMPL_HAVE_SPOT
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twa/bddprint.hh>
#include <spot/misc/minato.hh>
#include <spot/twa/formula2bdd.hh>
#include <typeinfo>
#endif
#include <boost/range/irange.hpp>
#include <unordered_map>
#include <unordered_set>
#include <boost/dynamic_bitset.hpp>
#include <ostream>
#include <limits>
#include <queue>
#include <vector>

// int ompl::control::Automaton::TransitionMap::eval(const World &w) const
// {
//     const auto d = entries.find(w);
//     if (d != entries.end())
//         return d->second;
//     for (const auto &entry : entries)
//     {
//         if (w.satisfies(entry.first))
//         {
//             // Since w satisfies another world that leads to d->second,
//             // we can add an edge directly from w to d->second.
//             entries[w] = entry.second;
//             return entry.second;
//         }
//     }
//     return -1;
// }

std::vector<int > ompl::control::Automaton::TransitionMap::eval(const World &w) const
{
    std::vector< int> states;
    const auto d = entries.find(w);
    if (d != entries.end())
        return d->second;
    for (const auto &entry : entries)
    {
        if (w.satisfies(entry.first))
        {
            // Since w satisfies another world that leads to d->second,
            // we can add an edge directly from w to d->second.
            entries[w] = entry.second;
            return entry.second;
        }
    }
    states.push_back(-1);
    return states;
}

ompl::control::Automaton::Automaton(unsigned int numProps, unsigned int numStates)
  : numProps_(numProps)
  , numStates_(numStates)
  , accepting_(numStates_, false)
  , transitions_(numStates_)
  , distances_(numStates_, std::numeric_limits<unsigned int>::max())
  , lowerbound_(numStates_, 0.0)
  , upperbound_(numStates_, 100.0)
{
}

#if OMPL_HAVE_SPOT
ompl::control::Automaton::Automaton(unsigned numProps, std::string formula, bool isCosafe)
  : Automaton::Automaton(numProps)
{
    if (!isCosafe)
        formula = "! (" + formula + ")";

    spot::formula f = spot::parse_formula(formula);
    spot::translator trans;

    // We want deterministic output (dfa)
    trans.set_pref(spot::postprocessor::Deterministic);
    // Apply all optimizations - will be slowest
    trans.set_level(spot::postprocessor::High);
    trans.set_type(spot::postprocessor::BA);
    spot::twa_graph_ptr au = trans.run(f);

    const auto &dict = au->get_dict();
    unsigned int n = au->num_states();
    for (unsigned int s = 0; s < n; ++s)
        addState(false);
    for (unsigned int s = 0; s < n; ++s)
    {
        // The out(s) method returns a fake container that can be
        // iterated over as if the contents was the edges going
        // out of s.  Each of these edge is a quadruplet
        // (src,dst,cond,acc).  Note that because this returns
        // a reference, the edge can also be modified.
        for (auto &t : au->out(s))
        {

            if (t.acc)
                setAccepting(s, true);

            // Parse the formula
            spot::minato_isop isop(t.cond);
            bdd clause = isop.next();
            if (clause == bddfalse)
                addTransition(s, numProps, t.dst);
            else
            {
                while (clause != bddfalse)
                {
                    ompl::control::World edgeLabel(numProps);
                    while (clause != bddtrue)
                    {
                        int var = bdd_var(clause);
                        const spot::bdd_dict::bdd_info &i = dict->bdd_map[var];
                        assert(i.type == spot::bdd_dict::var);
                        auto index = std::stoul(i.f.ap_name().substr(1));
                        bdd high = bdd_high(clause);
                        assert(index < numProps);
                        if (high == bddfalse)
                        {
                            edgeLabel[index] = false;
                            clause = bdd_low(clause);
                        }
                        else
                        {
                            assert(bdd_low(clause) == bddfalse);
                            edgeLabel[index] = true;
                            clause = high;
                        }
                    }
                    addTransition(s, edgeLabel, t.dst);

                    clause = isop.next();
                }
            }
        }
    }

    setStartState(au->get_init_state_number());

    if (!isCosafe)
        accepting_.flip();
}
#endif

unsigned int ompl::control::Automaton::addState(bool accepting)
{
    ++numStates_;
    accepting_.resize(numStates_);
    accepting_[numStates_ - 1] = accepting;
    transitions_.resize(numStates_);
    distances_.resize(numStates_, std::numeric_limits<unsigned int>::max());
    lowerbound_.resize(numStates_);
    lowerbound_[numStates_ - 1] = 0.0;
    upperbound_.resize(numStates_);
    upperbound_[numStates_ -1] = 100.0;
    return numStates_ - 1;
}

unsigned int ompl::control::Automaton::addState(bool accepting, double lowerbound, double upperbound)
{
    ++numStates_;
    accepting_.resize(numStates_);
    accepting_[numStates_ - 1] = accepting;
    transitions_.resize(numStates_);
    distances_.resize(numStates_, std::numeric_limits<unsigned int>::max());
    lowerbound_.resize(numStates_);
    lowerbound_[numStates_ - 1] = lowerbound;
    upperbound_.resize(numStates_);
    upperbound_[numStates_ -1] = upperbound;
    return numStates_ - 1;
}

void ompl::control::Automaton::setAccepting(unsigned int s, bool a)
{
    accepting_[s] = a;
}

bool ompl::control::Automaton::isAccepting(unsigned int s) const
{
    return accepting_[s];
}

void ompl::control::Automaton::setStartState(unsigned int s)
{
    startState_ = s;
    lowerbound_[0] = -1.0;
    upperbound_[0] = 100.0;
}

int ompl::control::Automaton::getStartState() const
{
    return startState_;
}

void ompl::control::Automaton::setTimeBound(unsigned int s, double lowerbound, double upperbound)
{
    lowerbound_[s] = lowerbound;
    upperbound_[s] = upperbound;
}

std::pair<double, double> ompl::control::Automaton::getTimeBound(int s)
{
    return std::make_pair(lowerbound_[s], upperbound_[s]);
}

// void ompl::control::Automaton::addTransition(unsigned int src, const World &w, unsigned int dest)
// {
//     TransitionMap &map = transitions_[src];
//     map.entries[w] = dest;
// }

void ompl::control::Automaton::addTransition(unsigned int src, const World &w, unsigned int dest)
{
    TransitionMap &map = transitions_[src];
    map.entries[w].push_back(dest);
}

bool ompl::control::Automaton::run(const std::vector<World> &trace) const
{ //FIX THIS
    int current = startState_;
    for (const auto &w : trace)
    {
        current = step(current, w)[0];
        if (current == -1)
            return false;
    }
    return true;
}

//FIX THIS BROKEN!
std::vector<int > ompl::control::Automaton::step(int state, const World &w) const
{   
    std::vector< int> states;
    if (state == -1){
        states.push_back(-1);
        return states;
    }
    return transitions_[state].eval(w);
}

ompl::control::Automaton::TransitionMap &ompl::control::Automaton::getTransitions(unsigned int src)
{
    return transitions_[src];
}

unsigned int ompl::control::Automaton::numStates() const
{
    return numStates_;
}

unsigned int ompl::control::Automaton::numTransitions() const
{
    unsigned int ntrans = 0;
    for (const auto &transition : transitions_)
        ntrans += transition.entries.size();
    return ntrans;
}

unsigned int ompl::control::Automaton::numProps() const
{
    return numProps_;
}

void ompl::control::Automaton::print(std::ostream &out) const
{
    out << "digraph automaton {" << std::endl;
    out << "rankdir=LR" << std::endl;
    for (unsigned int i = 0; i < numStates_; ++i)
    {
        out << i << R"( [label=")" << i << R"(",shape=)";
        out << (accepting_[i] ? "doublecircle" : "circle") << "]" << std::endl;

        for (const auto &e : transitions_[i].entries)
        {
            const World &w = e.first;
            std::vector<int> dests = e.second;
            for (auto dest : dests)
            {
            // unsigned int dest = e.second[0]; //FIX THIS
            const std::string formula = w.formula();
            out << i << " -> " << dest << R"( [label=")" << formula << R"("])" << std::endl;
            }
        }
    }
    out << "}" << std::endl;
}

unsigned int ompl::control::Automaton::distFromAccepting(unsigned int s) const
{
    if (distances_[s] < std::numeric_limits<unsigned int>::max())
        return distances_[s];
    if (accepting_[s])
        return 0;
    std::queue<unsigned int> q;
    std::unordered_set<unsigned int> processed;
    std::unordered_map<unsigned int, unsigned int> distance;

    q.push(s);
    distance[s] = 0;
    processed.insert(s);

    while (!q.empty())
    {
        unsigned int current = q.front();
        q.pop();
        if (accepting_[current])
        {
            distances_[s] = distance[current];
            return distance[current];
        }
        for (const auto &e : transitions_[current].entries)
        {
            unsigned int neighbor = e.second[0]; 
            if (processed.count(neighbor) > 0)
                continue;
            q.push(neighbor);
            processed.insert(neighbor);
            distance[neighbor] = distance[current] + 1;
        }
    }
    return std::numeric_limits<unsigned int>::max();
}

ompl::control::AutomatonPtr ompl::control::Automaton::AcceptingAutomaton(unsigned int numProps)
{
    auto phi(std::make_shared<Automaton>(numProps, 1));
    World trivial(numProps);
    phi->addTransition(0, trivial, 0);
    phi->setStartState(0);
    phi->setAccepting(0, true);
    // phi->lowerbound_[0] = -1.0;
    // phi->upperbound_[0] = 101.0;
    return phi;
}

ompl::control::AutomatonPtr ompl::control::Automaton::CoverageAutomaton(unsigned int numProps,
                                                                        const std::vector<unsigned int> &covProps)
{
    auto phi(std::make_shared<Automaton>(numProps, 1 << covProps.size()));
    for (unsigned int src = 0; src < phi->numStates(); ++src)
    {
        const boost::dynamic_bitset<> state(covProps.size(), src);
        World loop(numProps);
        // each value of p is an index of a proposition in covProps
        for (unsigned int p = 0; p < covProps.size(); ++p)
        {
            // if proposition covProps[p] has already been covered at state src, skip it
            if (state[p])
                continue;
            // for each proposition covProps[p] that has not yet been
            // covered at state src, construct a transition from src to (src|p)
            // on formula (covProps[p]==true)
            boost::dynamic_bitset<> target(state);
            target[p] = true;
            World nextProp(numProps);
            nextProp[covProps[p]] = true;
            phi->addTransition(src, nextProp, target.to_ulong());
            // also build a loop from src to src on formula with conjunct (covProps[p]==false)
            loop[covProps[p]] = false;
        }
        // now we add a loop from src to src on conjunction of (covProps[p]==false)
        // for every p such that the pth bit of src is 1
        phi->addTransition(src, loop, src);
    }
    phi->setAccepting(phi->numStates() - 1, true);
    phi->setStartState(0);
    return phi;
}

ompl::control::AutomatonPtr ompl::control::Automaton::SequenceAutomaton(unsigned int numProps,
                                                                        const std::vector<unsigned int> &seqProps)
{
    auto seq(std::make_shared<Automaton>(numProps, seqProps.size() + 1));
    for (unsigned int state = 0; state < seqProps.size(); ++state)
    {
        // loop when next proposition in sequence is not satisfied
        World loop(numProps);
        loop[seqProps[state]] = false;
        seq->addTransition(state, loop, state);
        // if (state == 1){
        //     World progress(numProps);
        //     progress[seqProps[state]] = true;
        //     seq->addTransition(state, progress, state);
        // }
        // progress forward when next proposition in sequence is satisfied
        World progress(numProps);
        progress[seqProps[state]] = true;
        seq->addTransition(state, progress, state + 1);

    }
    // loop on all input when in accepting state
    seq->addTransition(seqProps.size(), World(numProps), seqProps.size());
    seq->setAccepting(seqProps.size(), true);
    seq->setStartState(0);
    return seq;
}

ompl::control::AutomatonPtr ompl::control::Automaton::DisjunctionAutomaton(unsigned int numProps,
                                                                           const std::vector<unsigned int> &disjProps)
{
    auto disj(std::make_shared<Automaton>(numProps, 2));
    World loop(numProps);
    for (unsigned int disjProp : disjProps)
    {
        World satisfy(numProps);
        satisfy[disjProp] = true;
        loop[disjProp] = false;
        disj->addTransition(0, satisfy, 1);
    }
    disj->addTransition(0, loop, 0);
    disj->addTransition(1, World(numProps), 1);
    disj->setAccepting(1, true);
    disj->setStartState(0);
    return disj;
}

ompl::control::AutomatonPtr ompl::control::Automaton::AvoidanceAutomaton(unsigned int numProps,
                                                                         const std::vector<unsigned int> &avoidProps)
{
    /* An avoidance automaton is simply a disjunction automaton with its acceptance condition flipped. */
    AutomatonPtr avoid = DisjunctionAutomaton(numProps, avoidProps);
    avoid->setAccepting(0, true);
    avoid->setAccepting(1, false);
    return avoid;
}

ompl::control::AutomatonPtr ompl::control::Automaton::CoverageAutomaton(unsigned int numProps)
{
    const boost::integer_range<unsigned int> props = boost::irange(0u, numProps);
    return CoverageAutomaton(numProps, std::vector<unsigned int>(props.begin(), props.end()));
}


ompl::control::AutomatonPtr ompl::control::Automaton::SequenceAutomaton(unsigned int numProps)
{
    const boost::integer_range<unsigned int> props = boost::irange(0u, numProps);
    return SequenceAutomaton(numProps, std::vector<unsigned int>(props.begin(), props.end()));
}

ompl::control::AutomatonPtr ompl::control::Automaton::DisjunctionAutomaton(unsigned int numProps)
{
    const boost::integer_range<unsigned int> props = boost::irange(0u, numProps);
    return DisjunctionAutomaton(numProps, std::vector<unsigned int>(props.begin(), props.end()));
}

ompl::control::AutomatonPtr ompl::control::Automaton::MyAutomatonPhi2()
{
    unsigned int numProps = 3;
    auto seq(std::make_shared<Automaton>(numProps, 5));

    unsigned int state = 0;

    World loop(numProps);
    loop[1] = false;
    loop[2] = false;
    seq->addTransition(state, loop, state);

    World progress(numProps);
    progress[1] = true;
    progress[2] = false;
    seq->addTransition(state, progress, 1);

    World progress_2(numProps);
    progress_2[2] = true;
    progress_2[1] = false;
    seq->addTransition(state, progress_2, 2);

    World loopatone(numProps);
    loopatone[2] = false;
    seq->addTransition(1, loopatone, 1);

    World loopattwo(numProps);
    loopattwo[1] = false;
    seq->addTransition(2, loopattwo, 2);

    World progress1to3(numProps);
    progress1to3[2] = true;
    seq->addTransition(1, progress1to3, 3);

    World progress2to3(numProps);
    progress2to3[1] = true;
    seq->addTransition(2, progress2to3, 3);

    World loopat3(numProps);
    loopat3[3] = false;
    seq->addTransition(3, loopat3, 3);

    World progressto4(numProps);
    progressto4[3] = true;
    seq->addTransition(3, progressto4, 4);

    // loop on all input when in accepting state
    seq->addTransition(4, World(numProps), 4);
    seq->setAccepting(4, true);
    seq->setStartState(0);
    return seq;
}

ompl::control::AutomatonPtr ompl::control::Automaton::MyAutomatonPhi4()
{
    unsigned int numProps = 3;
    auto seq(std::make_shared<Automaton>(numProps, 9));

    unsigned int state = 0;

    World loop(numProps);
    loop[1] = false;
    loop[2] = false;
    seq->addTransition(state, loop, state);

    World progress(numProps);
    progress[1] = true;
    progress[2] = false;
    seq->addTransition(state, progress, 1);

    World progress_2(numProps);
    progress_2[2] = true;
    progress_2[1] = false;
    seq->addTransition(state, progress_2, 2);

    World loopatone(numProps);
    loopatone[2] = false;
    seq->addTransition(1, loopatone, 1);

    World loopattwo(numProps);
    loopattwo[1] = false;
    seq->addTransition(2, loopattwo, 2);

    World progress1to3(numProps);
    progress1to3[2] = true;
    seq->addTransition(1, progress1to3, 3);

    World progress2to3(numProps);
    progress2to3[1] = true;
    seq->addTransition(2, progress2to3, 3);

    World loopat3(numProps);
    loopat3[3] = false;
    seq->addTransition(3, loopat3, 3);

    World progressto4(numProps);
    progressto4[3] = true;
    seq->addTransition(3, progressto4, 4);

    seq->addTransition(4, loop, 4);
    seq->addTransition(4, progress, 5);
    seq->addTransition(4, progress_2, 6);
    seq->addTransition(5, loopatone, 5);
    seq->addTransition(6, loopattwo, 6);
    seq->addTransition(5, progress1to3, 7);
    seq->addTransition(6, progress2to3, 7);
    seq->addTransition(7, loopat3, 7);
    seq->addTransition(7, progressto4, 8);
    // loop on all input when in accepting state
    seq->addTransition(8, World(numProps), 8);
    seq->setAccepting(8, true);
    seq->setStartState(0);
    return seq;
}

ompl::control::AutomatonPtr ompl::control::Automaton::MyAutomatonPhi6()
{
    unsigned int numProps = 3;
    auto seq(std::make_shared<Automaton>(numProps, 6));

    unsigned int state = 0;

    World loop(numProps);
    loop[1] = false;
    seq->addTransition(state, loop, state);

    World progress(numProps);
    progress[2] = true;
    // progress[2] = false;
    seq->addTransition(state, progress, 1);

    World loopatone(numProps);
    // loopatone[4] = false;
    loopatone[3] = false;
    seq->addTransition(1, loopatone, 1);

    World progress2(numProps);
    progress2[3] = true;
    seq->addTransition(1, progress2, 2);

    World loopat2(numProps);
    loopat2[1] = false;
    loopat2[2] = false;
    seq->addTransition(2, loopat2, 2);

    World progress3(numProps);
    progress3[2] = true;
    seq->addTransition(2, progress3, 3);
    
    World loopat3(numProps);
    loopat3[1] = false;
    seq->addTransition(3, loopat3, 3);

    World progress2to4(numProps);
    progress2to4[1] = true;
    seq->addTransition(2, progress2to4, 4);

    World loopat4(numProps);
    loopat4[2] = false;
    seq->addTransition(4, loopat4, 4);

    World progress3to5(numProps);
    progress3to5[3] = true;
    seq->addTransition(3, progress3to5, 5);

    World progress4to5(numProps);
    progress4to5[3] = true;
    seq->addTransition(4, progress4to5, 5);

    // loop on all input when in accepting state
    seq->addTransition(5, World(numProps), 5);
    seq->setAccepting(5, true);
    seq->setStartState(0);
    return seq;
}

ompl::control::AutomatonPtr ompl::control::Automaton::MyAutomatonPhi5()
{
    unsigned int numProps = 4;
    auto seq(std::make_shared<Automaton>(numProps, 3));

    unsigned int state = 0;

    World loop(numProps);
    loop[1] = false;
    seq->addTransition(state, loop, state);

    World progress(numProps);
    progress[1] = true;
    progress[2] = false;
    seq->addTransition(state, progress, 1);

    World loopatone(numProps);
    loopatone[4] = false;
    loopatone[3] = false;
    seq->addTransition(1, loopatone, 1);

    World progress2(numProps);
    progress2[3] = true;
    progress[4] = false;
    seq->addTransition(1, progress2, 2);

    // loop on all input when in accepting state
    seq->addTransition(2, World(numProps), 2);
    seq->setAccepting(2, true);
    seq->setStartState(0);
    return seq;
}


ompl::control::AutomatonPtr ompl::control::Automaton::MyAutomatonPhi7()
{
    unsigned int numProps = 5;
    auto seq(std::make_shared<Automaton>(numProps, 4));

    unsigned int state = 0;

    World loop(numProps);
    loop[2] = false;
    seq->addTransition(state, loop, state);

    World progress(numProps);
    progress[2] = true;
    seq->addTransition(state, progress, 1);

    World loopatone(numProps);
    loopatone[4] = false;
    loopatone[5] = false;
    seq->addTransition(1, loopatone, 1);

    World progress1to2(numProps);
    progress1to2[4] = true;
    progress1to2[5] = false;
    seq->addTransition(1, progress1to2, 2);

    World loopattwo(numProps);
    loopattwo[3] = false;
    loopattwo[5] = false;
    seq->addTransition(2, loopattwo, 2);

    World progress2to3(numProps);
    progress2to3[3] = true;
    progress2to3[5] = false;
    seq->addTransition(2, progress2to3, 3);

    // loop on all input when in accepting state
    seq->addTransition(2, World(numProps), 2);
    seq->setAccepting(2, true);
    seq->setStartState(0);
    return seq;
}