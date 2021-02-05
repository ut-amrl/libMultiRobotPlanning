#pragma once

#include <vector>
#include "utils.hpp"

namespace libMultiRobotPlanning {

/*! \brief Represents the path for an agent

    This class is used to store the result of a planner for a single agent.
    It has both the ordered list of states that need to be traversed as well as
   the ordered
    list of actions together with their respective costs

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct PlanResult {
  //! states and their gScore
  std::vector<std::pair<State, Cost> > states;
  //! actions and their cost
  std::vector<std::pair<Action, Cost> > actions;
  //! actual cost of the result
  Cost cost;
  //! lower bound of the cost (for suboptimal solvers)
  Cost fmin;
};

template <typename State, typename Action, typename Cost>
std::ostream& operator<<(std::ostream& os,
                         const PlanResult<State, Action, Cost>& p) {
  os << "PlanResult: (cost: " << p.cost << ") [";
  if (p.states.empty()) {
    os << "]";
    return os;
  }
  NP_CHECK(p.states.size() == p.actions.size() + 1);
  os << "(" << p.states.front().first << ", " << p.states.front().second << ")";
  for (int i = 1; i < static_cast<int>(p.states.size()); ++i) {
    os << ", <" << p.actions[i - 1].first << ", " << p.actions[i - 1].second
       << ">, (" << p.states[i].first << ", " << p.states[i].second << ")";
  }
  os << "]";
  return os;
}

}  // namespace libMultiRobotPlanning
