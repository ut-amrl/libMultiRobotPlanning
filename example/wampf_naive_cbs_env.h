#pragma once

#include <unordered_set>

#include <boost/functional/hash.hpp>

#include <libMultiRobotPlanning/cbs.hpp>
#include <libMultiRobotPlanning/neighbor.hpp>
#include <libMultiRobotPlanning/planresult.hpp>
#include "wampf_individual.h"

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using Action = libMultiRobotPlanning::IndividualSpaceAction;
using State = libMultiRobotPlanning::State;
using Location = libMultiRobotPlanning::Location;

namespace naive_cbs_wampf_impl {
struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  bool operator==(const Conflict& o) {
    if (type != o.type) {
      return false;
    }

    if (type == Type::Vertex) {
      bool agents_equal = (agent1 == o.agent1 && agent2 == o.agent2) ||
                          (agent1 == o.agent2 && agent2 == o.agent1);
      return time == o.time && x1 == o.x1 && y1 == o.y1 && agents_equal;
    } else {
      bool agents_eq = (agent1 == o.agent1 && agent2 == o.agent2) ||
                       (agent1 == o.agent2 && agent2 == o.agent1);
      bool points_eq = (x1 == o.x1 && y1 == o.y1 && x2 == o.x2 && y2 == o.y2) ||
                       (x1 == o.x2 && y1 == o.y2 && x2 == o.x1 && y2 == o.y1);
      return time == o.time && agents_eq && points_eq;
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}

  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};
}  // namespace naive_cbs_wampf_impl

namespace std {
template <>
struct hash<naive_cbs_wampf_impl::VertexConstraint> {
  size_t operator()(const naive_cbs_wampf_impl::VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

namespace naive_cbs_wampf_impl {
struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}

  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& o) const {
    bool points_eq = (x1 == o.x1 && y1 == o.y1 && x2 == o.x2 && y2 == o.y2) ||
                     (x1 == o.x2 && y1 == o.y2 && x2 == o.x1 && y2 == o.y1);
    return time == o.time && points_eq;
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

}  // namespace naive_cbs_wampf_impl

namespace std {
template <>
struct hash<naive_cbs_wampf_impl::EdgeConstraint> {
  size_t operator()(const naive_cbs_wampf_impl::EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

namespace naive_cbs_wampf_impl {
struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) {
    std::vector<VertexConstraint> vertexIntersection;
    std::vector<EdgeConstraint> edgeIntersection;
    std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                          other.vertexConstraints.begin(),
                          other.vertexConstraints.end(),
                          std::back_inserter(vertexIntersection));
    std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                          other.edgeConstraints.begin(),
                          other.edgeConstraints.end(),
                          std::back_inserter(edgeIntersection));
    return !vertexIntersection.empty() || !edgeIntersection.empty();
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

}  // namespace naive_cbs_wampf_impl

namespace naive_cbs_wampf_impl {
template <typename Location>
class NaiveCBSEnvironment {
 public:
  NaiveCBSEnvironment(size_t dimx, size_t dimy,
                      std::unordered_set<Location> obstacles,
                      std::vector<Location> goals)
      : dimx_(dimx),
        dimy_(dimy),
        obstacles_(std::move(obstacles)),
        goals_(std::move(goals)) {}

  NaiveCBSEnvironment(const NaiveCBSEnvironment&) = delete;

  NaiveCBSEnvironment& operator=(const NaiveCBSEnvironment&) = delete;

  std::pair<Action, int> getActionsPad() { return {Action::Wait, 0}; }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int>>& solution,
      Conflict& result) const {
    // setting up entry_times, a vector that stores {entry time, agent idx}
    std::vector<std::pair<int, int>> entry_times;
    int last_exit = 0;
    for (size_t i = 0; i < solution.size(); i++) {
      NP_CHECK(solution[i].states.size() > 0);
      last_exit = std::max<int>(last_exit, solution[i].states[0].first.time +
                                               solution[i].actions.size());
      entry_times.push_back({solution[i].states[0].first.time, i});
    }
    std::sort(entry_times.begin(), entry_times.end());
    NP_CHECK(entry_times.size() > 1);
    NP_CHECK(entry_times[0].first <= entry_times[1].first);
    NP_CHECK(solution.size() > 1);
    NP_CHECK_EQ(entry_times.size(), solution.size());

    // stores the index of the current state inside each agent's
    // PlanResult.states in solution
    std::vector<int> state_idx_tracker(solution.size(), -1);

    // agent_idxs stores the set of agent indices that are in the window at time
    // t. updated as we iterate.
    std::vector<int> agent_idxs;

    int entry_time_i = 0;

    // if there is only one agent in the beginning of the window, we need to
    // add it to agent_idxs and update its state_idx_tracker value
    if (entry_times[0].first < entry_times[1].first) {
      agent_idxs.push_back(entry_times[0].second);
      state_idx_tracker[entry_times[0].second] =
          std::min<int>(entry_times[1].first - entry_times[0].first - 1,
                        solution[entry_times[0].second].states.size() - 1);
      entry_time_i++;
    }

    // Here, we iterate over each time step, checking collisions
    for (int t = entry_times[1].first; t <= last_exit; t++) {
      // adding agents to agent_idxs if they just entered the window
      while (entry_time_i < static_cast<int>(entry_times.size()) &&
             entry_times[entry_time_i].first == t) {
        agent_idxs.push_back(entry_times[entry_time_i].second);
        entry_time_i++;
      }

      // updating state_idx_tracker
      for (size_t i = 0; i < agent_idxs.size(); i++) {
        state_idx_tracker[agent_idxs[i]]++;
        // if an agent has reached its goal or left the window, we adjust
        // state_idx_tracker or remove it from agent_idxs
        NP_CHECK_LE(state_idx_tracker[agent_idxs[i]],
                    static_cast<int>(solution[agent_idxs[i]].states.size()));

        if (state_idx_tracker[agent_idxs[i]] >=
            static_cast<int>(solution[agent_idxs[i]].states.size())) {
          if (goals_[agent_idxs[i]].StateEquals(
                  solution[agent_idxs[i]].states.back().first)) {
            state_idx_tracker[agent_idxs[i]] =
                solution[agent_idxs[i]].states.size() - 1;
          } else {
            agent_idxs.erase(agent_idxs.begin() + i);
            i--;
          }
        }
      }

      // checking for vertex conflicts between agents in agent_idxs
      for (size_t i = 0; i < agent_idxs.size(); i++) {
        NP_CHECK_LT(state_idx_tracker[agent_idxs[i]],
                    static_cast<int>(solution[agent_idxs[i]].states.size()));
        const State& s1 = solution[agent_idxs[i]]
                              .states[state_idx_tracker[agent_idxs[i]]]
                              .first;
        for (size_t j = i + 1; j < agent_idxs.size(); j++) {
          const State& s2 = solution[agent_idxs[j]]
                                .states[state_idx_tracker[agent_idxs[j]]]
                                .first;
          if (s1 == s2 ||
              (goals_[agent_idxs[i]].StateEquals(s1) &&
               s1.equalExceptTime(s2)) ||
              (goals_[agent_idxs[j]].StateEquals(s2) &&
               s1.equalExceptTime(s2))) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = s1.x;
            result.y1 = s1.y;
            return true;
          }
        }
      }

      // checking for edge conflicts between agents in agent_idxs
      for (size_t i = 0; i < agent_idxs.size(); i++) {
        NP_CHECK_LT(state_idx_tracker[agent_idxs[i]],
                    static_cast<int>(solution[agent_idxs[i]].states.size()));
        // if this robot is at its window goal at time t, it cannot have an edge
        // conflict with another robot betwen t and t + 1
        if (state_idx_tracker[agent_idxs[i]] + 1 ==
            static_cast<int>(solution[agent_idxs[i]].states.size())) {
          continue;
        }
        const State& s1a = solution[agent_idxs[i]]
                               .states[state_idx_tracker[agent_idxs[i]]]
                               .first;
        const State& s1b = solution[agent_idxs[i]]
                               .states[state_idx_tracker[agent_idxs[i]] + 1]
                               .first;
        for (size_t j = i + 1; j < agent_idxs.size(); j++) {
          NP_CHECK(state_idx_tracker[agent_idxs[j]] <
                   static_cast<int>(solution[agent_idxs[j]].states.size()));
          // if this robot is at its window goal at time t, it cannot have an
          // edge conflict with another robot betwen t and t + 1
          if (state_idx_tracker[agent_idxs[j]] + 1 ==
              static_cast<int>(solution[agent_idxs[j]].states.size())) {
            continue;
          }
          const State& s2a = solution[agent_idxs[j]]
                                 .states[state_idx_tracker[agent_idxs[j]]]
                                 .first;
          const State& s2b = solution[agent_idxs[j]]
                                 .states[state_idx_tracker[agent_idxs[j]] + 1]
                                 .first;
          if ((s1a.x == s2b.x && s1a.y == s2b.y && s1a.time + 1 == s2b.time) &&
              (s2a.x == s1b.x && s2a.y == s1b.y && s2a.time + 1 == s1b.time)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = s1a.x;
            result.y1 = s1a.y;
            result.x2 = s1b.x;
            result.y2 = s1b.y;
            return true;
          }
        }
      }
    }
    return false;
  }

  bool getFirstConflictOld(
      const std::vector<PlanResult<State, Action, int>>& solution,
      Conflict& result) const {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State CBSState1 = GetCBSState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State CBSState2 = GetCBSState(j, solution, t);
          if (CBSState1.equalExceptTime(CBSState2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = CBSState1.x;
            result.y1 = CBSState1.y;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State CBSState1a = GetCBSState(i, solution, t);
        State CBSState1b = GetCBSState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State CBSState2a = GetCBSState(j, solution, t);
          State CBSState2b = GetCBSState(j, solution, t + 1);
          if (CBSState1a.equalExceptTime(CBSState2b) &&
              CBSState1b.equalExceptTime(CBSState2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = CBSState1a.x;
            result.y1 = CBSState1a.y;
            result.x2 = CBSState1b.x;
            result.y2 = CBSState1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict,
      std::map<size_t, Constraints>& constraints) const {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void createConstraintsFromConflict(const Conflict& conflict,
                                     std::map<size_t, Constraints>& constraints,
                                     int time_offset_agent1,
                                     int time_offset_agent2) const {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(VertexConstraint(
          conflict.time - time_offset_agent1, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.vertexConstraints.emplace(VertexConstraint(
          conflict.time - time_offset_agent2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(
          EdgeConstraint(conflict.time - time_offset_agent1, conflict.x1,
                         conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(
          EdgeConstraint(conflict.time - time_offset_agent2, conflict.x2,
                         conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void FixPadTime(std::vector<std::pair<State, int>>& sol) const {
    for (int i = 0; i < static_cast<int>(sol.size()); i++) {
      sol[i].first.time = i;
    }
  }

  bool CBSStateBoundsCheck(const State& s) const {
    return s.x >= 0 && s.x < dimx_ && s.y >= 0 && s.y < dimy_ &&
           obstacles_.find({s.x, s.y}) == obstacles_.end();
  }

  std::pair<int, int> GetDims() const { return {dimx_, dimy_}; }

 private:
  State GetCBSState(size_t agentIdx,
                    const std::vector<PlanResult<State, Action, int>>& solution,
                    size_t t) const {
    NP_CHECK(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    NP_CHECK(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

 private:
  int dimx_;
  int dimy_;
  std::unordered_set<Location> obstacles_;
  std::vector<Location> goals_;
};
}  // namespace naive_cbs_wampf_impl