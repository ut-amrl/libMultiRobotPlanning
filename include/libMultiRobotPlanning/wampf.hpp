#pragma once

#include <functional>
#include <vector>

#include "planresult.hpp"

namespace libMultiRobotPlanning {
namespace wampf {

template <typename State, typename Location, typename Action, typename Cost,
          typename Window, typename IndividualPlanner,
          typename WAMPFImplementation,
          typename StateHasher = std::hash<std::vector<State>>>
class WAMPF {
 public:
  using JointLoc = std::vector<Location>;
  using JointState = std::vector<State>;
  using JointPath = std::vector<PlanResult<State, Action, Cost>>;

 private:
  JointPath pi_;
  Cost individual_plan_cost_;
  std::vector<std::unique_ptr<Window>> W_;
  WAMPFImplementation impl_;

 public:
  WAMPF(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
        const JointState& start, const JointLoc& goal)
      : pi_(),
        individual_plan_cost_(0),
        W_(),
        impl_(dimx, dimy, obstacles, start, goal, &pi_) {
    pi_ = IndividualPlanner(dimx, dimy, obstacles, start, goal).Search();
    for (const auto& agent_path : pi_) {
      individual_plan_cost_ += agent_path.cost;
    }
  }

  std::tuple<const JointPath*, Cost, Cost> RecWAMPF() {
    for (int i = 0; i < static_cast<int>(W_.size()); ++i) {
      if (IsOverlappingAnotherWindow(*W_[i])) {
        std::unique_ptr<Window> wk_copy = std::move(W_[i]);
        W_.erase(W_.begin() + i);
        --i;
        PlanInOverlapWindows(std::move(wk_copy));
        continue;
      }
      impl_.GrowAndReplanIn(W_[i]);
    }

    auto collision_res = impl_.FirstCollisionWindow();
    while (collision_res) {
      PlanInOverlapWindows(std::move(*collision_res));
      collision_res = impl_.FirstCollisionWindow();
    }
    for (int i = 0; i < static_cast<int>(W_.size()); ++i) {
      if ((*W_[i]).ShouldQuit()) {
        W_.erase(W_.begin() + i);
        --i;
      }
    }

    if (W_.empty()) {
      return {&pi_, 1, 1};
    }

    Cost curr_plan_cost = 0;
    for (const auto& agent_path : pi_) {
      curr_plan_cost += agent_path.cost;
    }

    NP_CHECK_GE(curr_plan_cost, individual_plan_cost_);
    return {&pi_, curr_plan_cost, individual_plan_cost_};
  }

  const JointPath& GetPath() const { return pi_; }

  JointPath GetPath() { return pi_; }

 private:
  bool IsOverlappingAnotherWindow(const Window& wk) {
    for (const auto& wp : W_) {
      if (*wp != wk && wk.SuccessorOverlaps(*wp)) {
        return true;
      }
    }
    return false;
  }

  void PlanInOverlapWindows(std::unique_ptr<Window> w) {
    for (int i = 0; i < static_cast<int>(W_.size()); ++i) {
      if (w->Overlaps(*W_[i])) {
        w = w->Merge(*W_[i]);
        W_.erase(W_.begin() + i);
        --i;
      }
    }
    W_.emplace_back(std::move(w));
    impl_.PlanIn(W_.back());
  }
};

template <typename State, typename Action, typename Cost>
std::ostream& operator<<(
    std::ostream& os, const std::vector<PlanResult<State, Action, Cost>>& jp) {
  for (const auto& p : jp) {
    os << p << '\n';
  }
  return os;
}

}  // namespace wampf
}  // namespace libMultiRobotPlanning
