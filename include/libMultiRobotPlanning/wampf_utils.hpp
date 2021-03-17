#pragma once

#include <optional>
#include <unordered_map>

#include <libMultiRobotPlanning/planresult.hpp>
#include <libMultiRobotPlanning/utils.hpp>

namespace libMultiRobotPlanning {
namespace wampf {

// Replace region of current_path from repair_start to repair_end with repair.
// TODO: - Adjust times if the length of repair is greater than length of
// current
//         path
//       - NP_CHECK that the length of repair is greater than or equal to the
//         current path
template <typename State, typename Action>
PlanResult<State, Action, int> InsertPathRepair(
    PlanResult<State, Action, int> current_path,
    PlanResult<State, Action, int> repair, const int repair_start,
    const int repair_end) {
  NP_CHECK(static_cast<int>(current_path.states.size()) > repair_start);
  NP_CHECK_GT(static_cast<int>(current_path.states.size()), repair_end);
  NP_CHECK(repair_start < repair_end);
  NP_CHECK(!repair.states.empty());
  NP_CHECK(current_path.cost == current_path.states.back().second);
  NP_CHECK(repair.cost == repair.states.back().second);

  const auto repair_cost_delta =
      repair.cost - (current_path.states[repair_end].second -
                     current_path.states[repair_start].second);

  // Increase state costs in current path based on cost of repair.
  for (int i = repair_end + 1; i < static_cast<int>(current_path.states.size());
       ++i) {
    current_path.states[i].second += repair_cost_delta;
  }
  current_path.cost += repair_cost_delta;
  current_path.fmin += repair_cost_delta;

  // Increase state costs in repair to match current path entry.
  repair.states[0].second = current_path.states[repair_start].second;
  repair.states[0].first.time = current_path.states[repair_start].first.time;
  for (size_t i = 1; i < repair.states.size(); i++) {
    repair.states[i].second = repair.states[i - 1].second + 1;
    repair.states[i].first.time = repair.states[i - 1].first.time + 1;
  }

  const int current_state_start = repair_start + 1;
  const int current_state_end = repair_end;
  const int current_action_start = repair_start;
  const int current_action_end = repair_end - 1;

  current_path.states.erase(
      current_path.states.begin() + current_state_start,
      current_path.states.begin() + current_state_end + 1);
  current_path.actions.erase(
      current_path.actions.begin() + current_action_start,
      current_path.actions.begin() + current_action_end + 1);

  NP_CHECK(current_path.states[repair_start].first.equalExceptTime(
      repair.states.front().first));
  NP_CHECK(current_path.states[repair_start].second ==
           repair.states.front().second);

  current_path.states.insert(current_path.states.begin() + current_state_start,
                             repair.states.begin() + 1, repair.states.end());
  current_path.actions.insert(
      current_path.actions.begin() + current_action_start,
      repair.actions.begin(), repair.actions.end());

  for (size_t i = 1; i < current_path.states.size(); ++i) {
    const auto cost_delta =
        current_path.states[i].second - current_path.states[i - 1].second;
    NP_CHECK_EQ(cost_delta, 1);
  }

  NP_CHECK_EQ(current_path.states.size(), current_path.actions.size() + 1);

  return current_path;
}

// Assumes that the given path is one of the paths involved in the window.
template <typename State, typename Action, typename Window>
std::optional<std::pair<int, int>> GetWindowStartGoalIndex(
    const PlanResult<State, Action, int>& path, const Window& window) {
  int start_idx = -1;
  int end_idx = -1;
  for (int i = 0; i < static_cast<int>(path.states.size()); ++i) {
    const auto& s = path.states[i].first;
    if (!window.Contains(s)) {
      continue;
    }
    if (start_idx == -1) {
      start_idx = i;
    }
    end_idx = i;
  }
  if (start_idx != end_idx) {
    return {{start_idx, end_idx}};
  }
  // Either only one state in the window or both indexes are -1.
  // TODO: Ask Kyle why we don't include the robot if there is only one state
  //      in the window
  return {};
}

// this gives you a vector containing the indices of the first and last state
// of the agent in the window
template <typename State, typename Action, typename Window>
std::vector<std::pair<int, int>> GetWindowStartGoalIndexes(
    const std::vector<PlanResult<State, Action, int>>& joint_path,
    const Window& window) {
  std::vector<std::pair<int, int>> idxs;
  for (const auto& idx : window.agent_idxs_) {
    NP_CHECK(idx < joint_path.size());
    const auto& path = joint_path[idx];
    const auto res = GetWindowStartGoalIndex(path, window);
    NP_CHECK(res);
    idxs.emplace_back(std::move(*res));
  }
  return idxs;
}

template <typename State, typename Action, typename Window>
std::unordered_map<int, std::pair<int, int>> GetWindowStartGoalIndexMap(
    const std::vector<PlanResult<State, Action, int>>& joint_path,
    const Window& window) {
  std::unordered_map<int, std::pair<int, int>> idxs;
  for (const auto& idx : window.agent_idxs_) {
    NP_CHECK(idx < joint_path.size());
    const auto& path = joint_path[idx];
    const auto res = GetWindowStartGoalIndex(path, window);
    NP_CHECK(res);
    idxs.emplace(idx, std::move(*res));
  }
  return idxs;
}

}  // namespace wampf
}  // namespace libMultiRobotPlanning