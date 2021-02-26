#pragma once

#include <map>

#include "a_star_mod.hpp"
#include "utils.hpp"

namespace libMultiRobotPlanning {

/*
 * This is a modified CBS to allow for robots starting their paths at different
 * times. The main change is an addition of a vector of time offsets as a
 * parameter in the search() function.
 */

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBSMod {
 public:
  explicit CBSMod(Environment* environment) : m_env(environment) {}
  CBSMod(const CBSMod&) = default;
  CBSMod(CBSMod&&) = default;

  CBSMod& operator=(const CBSMod&) = default;
  CBSMod& operator=(CBSMod&&) = default;

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost>>& solution,
              const std::vector<int>& time_offsets) {
    NP_CHECK_EQ(time_offsets.size(), initialStates.size());
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(&llenv);
      bool success =
          lowLevel.search(initialStates[i], start.solution[i], time_offsets[i]);
      if (!success) {
        return false;
      }
      start.cost += start.solution[i].cost;
    }

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

    solution.clear();
    int id = 1;
    while (!open.empty()) {
      HighLevelNode P = open.top();
      m_env->onExpandHighLevelNode(P.cost);

      open.pop();

      Conflict conflict;
      if (!m_env->getFirstConflict(P.solution, conflict)) {
        // this gets rid of padding that we added in the most recent A* search
        for (int i = 0; i < static_cast<int>(P.solution.size()); i++) {
          while (P.solution[i].states.size() >
                 1 + P.solution[i].actions.size()) {
            P.solution[i].states.erase(P.solution[i].states.begin());
          }
        }
        solution = P.solution;
        return true;
      }

      // create additional nodes to resolve conflict
      std::map<size_t, Constraints> constraints;
      m_env->createConstraintsFromConflict(conflict, constraints,
                                           time_offsets[conflict.agent1],
                                           time_offsets[conflict.agent2]);
      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);

        newNode.cost -= newNode.solution[i].cost;

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(&llenv);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i],
                                       time_offsets[i]);

        newNode.cost += newNode.solution[i].cost;

        if (success) {
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        ++id;
      }
    }

    return false;
  }

 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment* env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env) {
      m_env->setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env->admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env->isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost>>& neighbors) {
      m_env->getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      m_env->onExpandLowLevelNode(s, fScore, gScore);
    }

    void FixPadTime(std::vector<std::pair<State, Cost>>& sol) {
      m_env->FixPadTime(sol);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

   private:
    Environment* m_env;
  };

 private:
  Environment* m_env;
  typedef AStarMod<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
