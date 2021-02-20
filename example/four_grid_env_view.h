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

  bool ShouldQuit() const {
    auto dims = env_->GetDims();
    return Contains({0, 0}) && Contains({dims.first, dims.second});
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

  bool SuccessorOverlaps(const FourConnectedEnvironmentView& o) const {
    return Overlaps({min_pos_.x - kRadiusGrowth, min_pos_.y - kRadiusGrowth},
                    {max_pos_.x + kRadiusGrowth, max_pos_.y + kRadiusGrowth},
                    o.min_pos_, o.max_pos_) &&
           Overlaps(
               {o.min_pos_.x - kRadiusGrowth, o.min_pos_.y - kRadiusGrowth},
               {o.max_pos_.x + kRadiusGrowth, o.max_pos_.y + kRadiusGrowth},
               min_pos_, max_pos_);
  }

  bool Overlaps(const FourConnectedEnvironmentView& o) const {
    return Overlaps(min_pos_, max_pos_, o.min_pos_, o.max_pos_) &&
           Overlaps(o.min_pos_, o.max_pos_, min_pos_, max_pos_);
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
    return env_->CBSStateBoundsCheck(s) && CBSStateWindowBoundsCheck(s) &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool TransitionValid(const CBSState& s1, const CBSState& s2) const {
    NP_NOT_NULL(constraints_);
    const auto& con = constraints_->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }
  bool Overlaps(const State& min_this, const State& max_this,
                const State& min_o, const State& max_o) const {
    return ((min_this.x <= max_o.x && max_this.x >= min_o.x) &&
            (min_this.y <= max_o.y && max_this.y >= min_o.y));
  }
};

}  // namespace naive_cbs_wampf_impl