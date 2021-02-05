#pragma once

#include <functional>
#include <vector>

#include "planresult.hpp"

namespace libMultiRobotPlanning {
namespace wampf {

template <typename State, typename Action, typename Cost, typename Window,
          typename IndividualPlanner, typename WAMPFImplementation,
          typename StateHasher = std::hash<std::vector<State>>>
class WAMPF {
 public:
  using JointState = std::vector<State>;
  using JointPath = std::vector<PlanResult<State, Action, Cost>>;

 private:
  JointPath pi_;
  std::vector<Window> W_;
  WAMPFImplementation impl_;

 public:
  WAMPF(size_t dimx, size_t dimy, std::unordered_set<State> obstacles,
        const JointState& start, const JointState& goal)
      : pi_(), W_(), impl_(dimx, dimy, obstacles, start, goal, &pi_) {
    pi_ = IndividualPlanner(dimx, dimy, obstacles, start, goal).Search();
    W_ = {};
    std::cout << "Individual space plan: " << pi_;
  }

  void RecWAMPF() {
    for (int i = 0; i < static_cast<int>(W_.size()); ++i) {
      auto& wk = W_[i];
      if (IsOverlappingAnotherWindow(wk)) {
        auto wk_copy = std::move(wk);
        W_.erase(W_.begin() + i);
        --i;
        PlanInOverlapWindows(std::move(wk_copy));
        continue;
      }
      impl_.GrowAndReplanIn(&wk);
    }

    auto collision_res = impl_.FirstCollisionWindow();
    while (collision_res) {
      W_.emplace_back(std::move(*collision_res));
      PlanInOverlapWindows(W_.back());
      collision_res = impl_.FirstCollisionWindow();
    }

    for (int i = 0; i < static_cast<int>(W_.size()); ++i) {
      if (W_[i].ShouldQuit()) {
        W_.erase(W_.begin() + i);
        --i;
      }
    }

    // Report pi_;
  }

  const JointPath& GetPath() const { return pi_; }

  JointPath GetPath() { return pi_; }

 private:
  bool IsOverlappingAnotherWindow(const Window& wk) {
    for (const auto& wp : W_) {
      if (wp != wk && wk.SuccessorOverlaps(wp)) {
        return true;
      }
    }
    return false;
  }

  void PlanInOverlapWindows(Window w) {
    for (int i = 0; i < static_cast<int>(W_.size()); ++i) {
      if (w.Overlaps(W_[i])) {
        w = w.Merge(W_[i]);
        W_.erase(W_.begin() + i);
        --i;
      }
    }
    W_.emplace_back(std::move(w));
    impl_.PlanIn(&W_.back());
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