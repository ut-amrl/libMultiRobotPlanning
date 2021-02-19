#pragma once

#include <unordered_set>

#include <boost/functional/hash.hpp>

#include <libMultiRobotPlanning/cbs.hpp>
#include <libMultiRobotPlanning/neighbor.hpp>
#include <libMultiRobotPlanning/planresult.hpp>
#include "wampf_environment_view.h"
#include "wampf_naive_cbs_env.h"
#include "wampf_state.h"

using State = libMultiRobotPlanning::State;

namespace naive_cbs_wampf_impl {
template <typename Environment, int kStartRadius = 2, int kRadiusGrowth = 1>
class FourConnectedEnvironmentView
    : naive_cbs_wampf_impl::EnvironmentView<Environment, State> {
 private:
  const Environment* env_;
  std::vector<State> goals_;
  size_t agentIdx_;
  const Constraints* constraints_;
  int lastGoalConstraint_;
  int highLevelExpanded_;
  int lowLevelExpanded_;

 public:
  State min_pos_;
  State max_pos_;

  void UpdateGoals(std::vector<State> goals) { goals_ = goals; }

  FourConnectedEnvironmentView(State pos, std::vector<State> goals,
                               const Environment* environment)
      : env_(environment),
        goals_(std::move(goals)),
        agentIdx_(0),
        constraints_(nullptr),
        lastGoalConstraint_(-1),
        highLevelExpanded_(0),
        lowLevelExpanded_(0),
        min_pos_(pos),
        max_pos_(pos) {
    min_pos_.x -= kStartRadius;
    min_pos_.y -= kStartRadius;
    max_pos_.x += kStartRadius;
    max_pos_.y += kStartRadius;
  }

  FourConnectedEnvironmentView(State min_pos, State max_pos,
                               std::vector<State> goals,
                               const Environment* environment)
      : env_(environment),
        goals_(std::move(goals)),
        agentIdx_(0),
        constraints_(nullptr),
        lastGoalConstraint_(-1),
        highLevelExpanded_(0),
        lowLevelExpanded_(0),
        min_pos_(min_pos),
        max_pos_(max_pos) {}

  bool operator==(const FourConnectedEnvironmentView& other) const {
    return (min_pos_ == other.min_pos_) && (max_pos_ == other.max_pos_);
  }

  FourConnectedEnvironmentView(const FourConnectedEnvironmentView&) = default;
  FourConnectedEnvironmentView(FourConnectedEnvironmentView&&) = default;

  FourConnectedEnvironmentView& operator=(
      const FourConnectedEnvironmentView& o) = default;
  FourConnectedEnvironmentView& operator=(FourConnectedEnvironmentView&) =
      default;

  int admissibleHeuristic(const CBSState& s) const {
    NP_CHECK_LT(agentIdx_, goals_.size());
    return std::abs(s.x - goals_[agentIdx_].x) +
           std::abs(s.y - goals_[agentIdx_].y);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const FourConnectedEnvironmentView& c) {
    return os << "min_pos_: " << c.min_pos_ << ", max_pos_: " << c.max_pos_;
  }

  bool Contains(const State& s) const {
    return ((s.x >= min_pos_.x && s.x <= max_pos_.x) &&
            (s.y >= min_pos_.y && s.y <= max_pos_.y));
  }

  void Grow() {
    min_pos_.x -= kRadiusGrowth;
    min_pos_.y -= kRadiusGrowth;
    max_pos_.x += kRadiusGrowth;
    max_pos_.y += kRadiusGrowth;
  }

  bool SuccessorOverlaps(const FourConnectedEnvironmentView& other) const {
    State min_growth(min_pos_.x - kRadiusGrowth, min_pos_.y - kRadiusGrowth);
    State max_growth(max_pos_.x + kRadiusGrowth, max_pos_.y + kRadiusGrowth);
    State off1(min_pos_.x - kRadiusGrowth, max_pos_.y + kRadiusGrowth);
    State off2(max_pos_.x + kRadiusGrowth, min_pos_.y - kRadiusGrowth);

    // Check if our four corners are inside their box.
    if (other.Contains(min_growth) || other.Contains(max_growth) ||
        other.Contains(off1) || other.Contains(off2)) {
      return true;
    }

    // Inflating their box is equivalent to inflating our box.
    State other_off1(other.min_pos_.x - kRadiusGrowth,
                     other.max_pos_.y + kRadiusGrowth);
    State other_off2(other.max_pos_.x + kRadiusGrowth,
                     other.min_pos_.y - kRadiusGrowth);

    // Check if their four corners are inside our box.
    if (Contains(other.min_pos_) || Contains(other.max_pos_) ||
        Contains(other_off1) || Contains(other_off2)) {
      return true;
    }
    return false;
  }

  bool Overlaps(const FourConnectedEnvironmentView& other) const {
    State off1(min_pos_.x, max_pos_.y);
    State off2(max_pos_.x, min_pos_.y);

    // Check if our four corners are inside their box.
    if (other.Contains(min_pos_) || other.Contains(max_pos_) ||
        other.Contains(off1) || other.Contains(off2)) {
      return true;
    }

    State other_off1(other.min_pos_.x, other.max_pos_.y);
    State other_off2(other.max_pos_.x, other.min_pos_.y);

    // Check if their four corners are inside our box.
    if (Contains(other.min_pos_) || Contains(other.max_pos_) ||
        Contains(other_off1) || Contains(other_off2)) {
      return true;
    }
    return false;
  }

  FourConnectedEnvironmentView Merge(
      const FourConnectedEnvironmentView& o) const {
    int min_x = std::min(min_pos_.x, o.min_pos_.x);
    int max_x = std::max(max_pos_.x, o.max_pos_.x);
    int min_y = std::min(min_pos_.y, o.min_pos_.y);
    int max_y = std::max(max_pos_.y, o.max_pos_.y);

    return {State(min_x, min_y), State(max_x, max_y), std::vector<State>(),
            env_};
  }

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    NP_NOT_NULL(constraints);
    agentIdx_ = agentIdx;
    constraints_ = constraints;
    lastGoalConstraint_ = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == goals_[agentIdx_].x && vc.y == goals_[agentIdx_].y) {
        lastGoalConstraint_ = std::max(lastGoalConstraint_, vc.time);
      }
    }
  }

  bool isSolution(const CBSState& s) const {
    NP_CHECK_LT(agentIdx_, goals_.size());
    return s.x == goals_[agentIdx_].x && s.y == goals_[agentIdx_].y &&
           s.time > lastGoalConstraint_;
  }

  void getNeighbors(
      const CBSState& s,
      std::vector<Neighbor<CBSState, CBSAction, int>>& neighbors) const {
    neighbors.clear();
    {
      CBSState n(s.time + 1, s.x, s.y);
      //      printf("Entering wait\n");
      //      if (!CBSStateValid(n)) printf("CBSStateValid wait is false\n");
      //      if (!TransitionValid(s, n)) printf("TransitionValid wait is
      //      false\n"); printf("leafing wait\n");
      if (CBSStateValid(n) && TransitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Wait, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x - 1, s.y);
      if (CBSStateValid(n) && TransitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Left, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x + 1, s.y);
      //      printf("Entering right\n");
      //      if (!CBSStateValid(n)) printf("CBSStateValid right is false\n");
      //      if (!TransitionValid(s, n)) printf("TransitionValid right is
      //      false\n"); printf("leafing right\n");

      if (CBSStateValid(n) && TransitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Right, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x, s.y + 1);
      if (CBSStateValid(n) && TransitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Up, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x, s.y - 1);
      if (CBSStateValid(n) && TransitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<CBSState, CBSAction, int>>& solution,
      Conflict& result) const {
    return env_->getFirstConflict(solution, result);
  }

  void createConstraintsFromConflict(
      const Conflict& conflict,
      std::map<size_t, Constraints>& constraints) const {
    env_->createConstraintsFromConflict(conflict, constraints);
  }

  void onExpandHighLevelNode(int /*cost*/) { highLevelExpanded_++; }

  void onExpandLowLevelNode(const CBSState& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    lowLevelExpanded_++;
  }

 private:
  bool CBSStateWindowBoundsCheck(const CBSState& s) const {
    return s.x >= min_pos_.x && s.x <= max_pos_.x && s.y >= min_pos_.y &&
           s.y <= max_pos_.y;
  }

  bool CBSStateValid(const CBSState& s) const {
    NP_NOT_NULL(constraints_);
    const auto& con = constraints_->vertexConstraints;
    //    if (!env_->CBSStateBoundsCheck(s)) printf("env->CBSStateBoundsCheck is
    //    false\n"); if (!CBSStateWindowBoundsCheck(s))
    //    printf("CBSStateWindowBoundsCheck is false\n"); if
    //    (!(con.find(VertexConstraint(s.time, s.x, s.y)) == con.end()))
    //    printf("vertex constraint found\n");
    return env_->CBSStateBoundsCheck(s) && CBSStateWindowBoundsCheck(s) &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool TransitionValid(const CBSState& s1, const CBSState& s2) const {
    NP_NOT_NULL(constraints_);
    const auto& con = constraints_->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }
};

}  // namespace naive_cbs_wampf_impl