#pragma once

#include <map>

#include "a_star.hpp"

namespace libMultiRobotPlanning {

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBS {
 public:
  explicit CBS(Environment* environment) : m_env(environment) {}
  CBS(const CBS&) = default;
  CBS(CBS&&) = default;

  CBS& operator=(const CBS&) = default;
  CBS& operator=(CBS&&) = default;

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(&llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i]);
      if (!success) {
        return false;
      }
      start.cost += start.solution[i].cost;
    }

    // std::priority_queue<HighLevelNode> open;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
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
        std::cout << "done; cost: " << P.cost << std::endl;
        solution = P.solution;
        return true;
      }

      // create additional nodes to resolve conflict
      std::map<size_t, Constraints> constraints;
      m_env->createConstraintsFromConflict(conflict, constraints);
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
        bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

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
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
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
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      m_env->getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      m_env->onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

   private:
    Environment* m_env;
  };

 private:
  Environment* m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
