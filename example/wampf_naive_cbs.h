#pragma once

#include <algorithm>
#include <vector>

#include <libMultiRobotPlanning/cbs.hpp>
#include <libMultiRobotPlanning/wampf_utils.hpp>

#include "four_grid_env_view.h"
#include "wampf_individual.h"
#include "wampf_naive_cbs_env.h"
#include "wampf_state.h"
#include "wampf_window.h"

namespace wampf_impl {

using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::IndividualSpaceAStar;
using libMultiRobotPlanning::IndividualSpaceEnvironment;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;

using Cost = int;
using JointState = std::vector<State>;
using JointPath = std::vector<PlanResult<State, IndividualSpaceAction, Cost>>;
using Env = naive_cbs_wampf_impl::NaiveCBSEnvironment<State>;
using EnvView = naive_cbs_wampf_impl::FourConnectedEnvironmentView<
    naive_cbs_wampf_impl::NaiveCBSEnvironment<State>>;

using Window = libMultiRobotPlanning::Window<Env, EnvView>;

class NaiveACBSImplementation {
 private:
  Env env_;
  JointState start_;
  JointPath* path_;

 public:
  NaiveACBSImplementation(size_t dimx, size_t dimy,
                          std::unordered_set<State> obstacles,
                          const JointState& start, const JointState& goal,
                          JointPath* path)
      : env_(dimx, dimy, obstacles, goal), start_(start), path_(path) {}

  std::optional<std::unique_ptr<Window>> FirstCollisionWindow() {
    auto res = FirstConflict();
    if (!res) {
      return {};
    }

    return {std::make_unique<Window>(std::move(res->first),
                                     std::move(res->second), &env_)};
  }

  // TODO: compute the offset
  std::pair<std::vector<int>, std::vector<std::pair<int, int>>>
  GetStartEndIndices(const Window& w) const {
    // Start and goal indices in the order of the index list contained in the
    // window.
    const auto start_goal_idxs =
        libMultiRobotPlanning::wampf::GetWindowStartGoalIndexes(*path_, w);
    NP_CHECK(!start_goal_idxs.empty());
    NP_CHECK_GE(start_goal_idxs.size(), 2);

    std::vector<int> time_offsets;

    int min_start_idx = start_goal_idxs.front().first;
    //    int max_goal_idx = start_goal_idxs.front().second;
    for (const auto& start_goal : start_goal_idxs) {
      min_start_idx = std::min(min_start_idx, start_goal.first);
      //      max_goal_idx = std::max(max_goal_idx, start_goal.second);
    }
    for (size_t i = 0; i < start_goal_idxs.size(); i++) {
      time_offsets.push_back(start_goal_idxs[i].first - min_start_idx);
    }
    NP_CHECK_EQ(start_goal_idxs.size(), time_offsets.size());
    // Verify that each start index is within the range of the path.
    for (const auto& idx : w.agent_idxs_) {
      NP_CHECK(idx < path_->size());
      NP_CHECK_LT(min_start_idx, static_cast<int>((*path_)[idx].states.size()));
    }
    return {time_offsets, start_goal_idxs};
  }

  // TODO: fix start_state and goal_state, fix InsertPathRepair to work with new
  // format of repairs
  void PlanIn(std::unique_ptr<Window>& w) {
    auto [time_offsets, start_goal_idxs] = GetStartEndIndices(*w);
    NP_CHECK_EQ(start_goal_idxs.size(), time_offsets.size());

    std::vector<naive_cbs_wampf_impl::CBSState> start_state;
    JointState goal_state;
    for (size_t i = 0; i < w->agent_idxs_.size(); i++) {
      const auto start = GetState(w->agent_idxs_[i], start_goal_idxs[i].first);
      start_state.push_back(
          naive_cbs_wampf_impl::CBSState(0, start.x, start.y));
      goal_state.emplace_back(
          GetState(w->agent_idxs_[i], start_goal_idxs[i].second));
    }

    w->env_view_.UpdateGoals(goal_state);

    std::vector<PlanResult<naive_cbs_wampf_impl::CBSState,
                           naive_cbs_wampf_impl::CBSAction, int>>
        cbs_repair;

    bool success = w->cbs_.search(start_state, cbs_repair, time_offsets);
    if (!success) {
      w->Grow();
      PlanIn(w);
    }
    auto repair = CBSRepairToStandardRepair(cbs_repair);

    // you need to pad when goal_idx - start_idx is greater than the size of
    // the states vector for an agent's repair
    for (size_t i = 0; i < repair.size(); i++) {
      int required_len = start_goal_idxs[i].second - start_goal_idxs[i].first;
      if (required_len > static_cast<int>(repair[i].states.size())) {
        PadPlanResult(repair[i], required_len);
      }
    }

    for (size_t i = 0; i < repair.size(); ++i) {
      NP_CHECK(i < w->agent_idxs_.size());
      const size_t path_idx = w->agent_idxs_[i];
      NP_CHECK(path_idx < path_->size());
      auto& local_path = (*path_)[path_idx];
      local_path = libMultiRobotPlanning::wampf::InsertPathRepair(
          local_path, repair[i], start_goal_idxs[i].first,
          start_goal_idxs[i].second);
    }
  }

  void GrowAndReplanIn(std::unique_ptr<Window>& w) {
    w->Grow();
    PlanIn(w);
  }

 private:
  void PadPlanResult(PlanResult<State, IndividualSpaceAction, int>& path,
                     int required_len) {
    while (static_cast<int>(path.states.size()) < required_len) {
      auto back_state_copy = path.states.back();
      path.states.emplace_back(back_state_copy);
      path.actions.push_back({IndividualSpaceAction::Wait, 1});
      path.cost += 1;
    }
  }

  IndividualSpaceAction CBSActionToIndividualSpaceAction(
      const naive_cbs_wampf_impl::CBSAction& a) const {
    switch (a) {
      case naive_cbs_wampf_impl::CBSAction::Up:
        return IndividualSpaceAction::Up;
      case naive_cbs_wampf_impl::CBSAction::Down:
        return IndividualSpaceAction::Down;
      case naive_cbs_wampf_impl::CBSAction::Left:
        return IndividualSpaceAction::Left;
      case naive_cbs_wampf_impl::CBSAction::Right:
        return IndividualSpaceAction::Right;
      case naive_cbs_wampf_impl::CBSAction::Wait:
        return IndividualSpaceAction::Wait;
    }
    return IndividualSpaceAction::Wait;
  }

  std::vector<PlanResult<State, IndividualSpaceAction, int>>
  CBSRepairToStandardRepair(
      const std::vector<PlanResult<naive_cbs_wampf_impl::CBSState,
                                   naive_cbs_wampf_impl::CBSAction, int>>&
          cbs_repair) {
    std::vector<PlanResult<State, IndividualSpaceAction, int>> repair;
    int max_agent_len = cbs_repair[0].states.size();
    for (const auto& r : cbs_repair) {
      max_agent_len =
          std::max(static_cast<int>(r.states.size()), max_agent_len);
      repair.push_back({});
      for (const auto& s : r.states) {
        repair.back().states.push_back({{s.first.x, s.first.y}, s.second});
      }
      for (const auto& a : r.actions) {
        repair.back().actions.push_back(
            {CBSActionToIndividualSpaceAction(a.first), a.second});
      }
      repair.back().cost = r.cost;
      repair.back().fmin = r.fmin;
    }
    return repair;
  }

  State GetState(size_t agent_idx, Cost timestep) const {
    NP_CHECK(agent_idx < (*path_).size());
    const auto& agent_path = (*path_)[agent_idx];
    if (timestep < static_cast<int>(agent_path.states.size())) {
      NP_CHECK(agent_path.states[timestep].second == timestep);
      return agent_path.states[timestep].first;
    }
    NP_CHECK(!agent_path.states.empty());
    return agent_path.states.back().first;
  }

  std::optional<std::pair<State, std::vector<size_t>>> FirstConflict() {
    int max_t = 0;
    for (const auto& sol : (*path_)) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t agent_i = 0; agent_i < (*path_).size(); ++agent_i) {
        State state_i = GetState(agent_i, t);
        for (size_t agent_j = agent_i + 1; agent_j < (*path_).size();
             ++agent_j) {
          State state_j = GetState(agent_j, t);
          if (state_i == state_j) {
            return {{state_i, {agent_i, agent_j}}};
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t agent_i = 0; agent_i < (*path_).size(); ++agent_i) {
        State state_i_t = GetState(agent_i, t);
        State state_i_tp1 = GetState(agent_i, t + 1);
        for (size_t agent_j = agent_i + 1; agent_j < (*path_).size();
             ++agent_j) {
          State state_j_t = GetState(agent_j, t);
          State state_j_tp1 = GetState(agent_j, t + 1);
          if (state_i_t == state_j_tp1 && state_i_tp1 == state_j_t) {
            return {{state_i_tp1, {agent_i, agent_j}}};
          }
        }
      }
    }
    return {};
  }
};

}  // namespace wampf_impl