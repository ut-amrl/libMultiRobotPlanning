#pragma once

#include <iostream>
#include <unordered_set>
#include <vector>

#include <libMultiRobotPlanning/neighbor.hpp>

#include "wampf_state.h"

namespace libMultiRobotPlanning {

enum class IndividualSpaceAction { Up, Down, Left, Right, Wait };

std::ostream& operator<<(std::ostream& os, const IndividualSpaceAction& a) {
  switch (a) {
    case IndividualSpaceAction::Up:
      os << "Up";
      break;
    case IndividualSpaceAction::Down:
      os << "Down";
      break;
    case IndividualSpaceAction::Left:
      os << "Left";
      break;
    case IndividualSpaceAction::Right:
      os << "Right";
      break;
    case IndividualSpaceAction::Wait:
      os << "Wait";
      break;
  }
  return os;
}

class IndividualSpaceEnvironment {
 public:
  IndividualSpaceEnvironment(size_t dimx, size_t dimy,
                             std::unordered_set<State> obstacles, State goal)
      : dimx_(dimx),
        dimy_(dimy),
        obstacles_(std::move(obstacles)),
        goal_(std::move(goal))  // NOLINT
  {}

  int admissibleHeuristic(const State& s) {
    return std::abs(s.x - goal_.x) + std::abs(s.y - goal_.y);
  }

  bool isSolution(const State& s) { return s == goal_; }

  void getNeighbors(
      const State& s,
      std::vector<Neighbor<State, IndividualSpaceAction, int>>& neighbors) {
    neighbors.clear();

    State up(s.x, s.y + 1);
    if (stateValid(up)) {
      neighbors.emplace_back(Neighbor<State, IndividualSpaceAction, int>(
          up, IndividualSpaceAction::Up, 1));
    }
    State down(s.x, s.y - 1);
    if (stateValid(down)) {
      neighbors.emplace_back(Neighbor<State, IndividualSpaceAction, int>(
          down, IndividualSpaceAction::Down, 1));
    }
    State left(s.x - 1, s.y);
    if (stateValid(left)) {
      neighbors.emplace_back(Neighbor<State, IndividualSpaceAction, int>(
          left, IndividualSpaceAction::Left, 1));
    }
    State right(s.x + 1, s.y);
    if (stateValid(right)) {
      neighbors.emplace_back(Neighbor<State, IndividualSpaceAction, int>(
          right, IndividualSpaceAction::Right, 1));
    }
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

  bool stateValid(const State& s) {
    return s.x >= 0 && s.x < dimx_ && s.y >= 0 && s.y < dimy_ &&
           obstacles_.find(s) == obstacles_.end();
  }

 private:
  int dimx_;
  int dimy_;
  std::unordered_set<State> obstacles_;
  State goal_;
};
}  // namespace libMultiRobotPlanning