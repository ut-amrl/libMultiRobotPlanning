#pragma once

#include "a_star.hpp"
#include "neighbor.hpp"
#include "planresult.hpp"
#include "utils.hpp"

namespace libMultiRobotPlanning {

template <typename State, typename Action, typename Cost, typename Environment,
          typename StateHasher = std::hash<State>>
struct IndividualSpaceAStar {
  using JointState = std::vector<State>;

 private:
  size_t dimx_;
  size_t dimy_;
  std::unordered_set<State> obstacles_;
  JointState joint_start_;
  JointState joint_goal_;

 public:
  IndividualSpaceAStar(size_t dimx, size_t dimy,
                       std::unordered_set<State> obstacles,
                       JointState joint_start, JointState joint_goal)
      : dimx_(dimx),
        dimy_(dimy),
        obstacles_(std::move(obstacles)),
        joint_start_(std::move(joint_start)),
        joint_goal_(std::move(joint_goal)) {
    CHECK(joint_start_.size() == joint_goal_.size());
  }

  std::vector<PlanResult<State, Action, Cost>> Search() {
    std::vector<PlanResult<State, Action, Cost>> results;
    for (size_t i = 0; i < joint_goal_.size(); ++i) {
      const State& start = joint_start_[i];
      const State& goal = joint_goal_[i];
      Environment env(dimx_, dimy_, obstacles_, goal);
      AStar<State, Action, int, Environment> astar(env);
      PlanResult<State, Action, Cost> solution;
      if (env.stateValid(start)) {
        astar.search(start, solution);
      }
      results.emplace_back(std::move(solution));
    }
    return results;
  }
};
}  // namespace libMultiRobotPlanning