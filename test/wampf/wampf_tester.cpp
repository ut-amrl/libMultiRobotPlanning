#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_set>

#include <gtest/gtest.h>

#include <libMultiRobotPlanning/wampf_utils.hpp>
#include "../../example/four_grid_env_view.h"
#include "../../example/wampf_individual.h"
#include "../../example/wampf_state.h"
#include "../../example/wampf_window.h"

using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;
using libMultiRobotPlanning::Window;
using libMultiRobotPlanning::wampf::GetWindowStartGoalIndexes;
using libMultiRobotPlanning::wampf::InsertPathRepair;
using PR = PlanResult<State, IndividualSpaceAction, int>;
using Action = IndividualSpaceAction;
using naive_cbs_wampf_impl::CBSAction;
using naive_cbs_wampf_impl::CBSState;
using naive_cbs_wampf_impl::Conflict;
using naive_cbs_wampf_impl::Constraints;
using naive_cbs_wampf_impl::FourConnectedEnvironmentView;
using naive_cbs_wampf_impl::NaiveCBSEnvironment;

void VerifyPathIntegrity(const PR& result) {
  EXPECT_EQ(result.actions.size() + 1, result.states.size());
  EXPECT_EQ(result.fmin, result.cost);
  for (size_t i = 1; i < result.states.size(); ++i) {
    const auto state_i_tuple = result.states[i];
    const auto state_im1_tuple = result.states[i - 1];
    EXPECT_EQ(1, state_i_tuple.second - state_im1_tuple.second);
    const auto state_i = state_i_tuple.first;
    const auto state_im1 = state_im1_tuple.first;
    EXPECT_LT(i - 1, result.actions.size());
    const auto action = result.actions[i - 1].first;
    switch (action) {
      case Action::Up: {
        EXPECT_EQ(state_im1.x, state_i.x);
        EXPECT_EQ(state_im1.y + 1, state_i.y);
        break;
      }
      case Action::Down: {
        EXPECT_EQ(state_im1.x, state_i.x);
        EXPECT_EQ(state_im1.y - 1, state_i.y);
        break;
      }
      case Action::Left: {
        EXPECT_EQ(state_im1.x - 1, state_i.x);
        EXPECT_EQ(state_im1.y, state_i.y);
        break;
      }
      case Action::Right: {
        EXPECT_EQ(state_im1.x + 1, state_i.x);
        EXPECT_EQ(state_im1.y, state_i.y);
        break;
      }
      case Action::Wait: {
        EXPECT_EQ(state_im1.x, state_i.x);
        EXPECT_EQ(state_im1.y, state_i.y);
        break;
      }
    }
  }
}

TEST(InsertLongerRepair, StraightToUpDown) {
  PR full_path;
  full_path.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{0, 2}, 2},
                      {{0, 3}, 3}, {{0, 4}, 4}, {{0, 5}, 5}};
  full_path.actions = {{Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1}};
  full_path.cost = 5;
  full_path.fmin = 5;

  PR repair;
  repair.states = {{{0, 1}, 0}, {{1, 1}, 1}, {{1, 2}, 2}, {{0, 2}, 3}};
  repair.actions = {{Action::Right, 1}, {Action::Up, 1}, {Action::Left, 1}};
  repair.cost = 3;
  repair.fmin = 3;

  int repair_start = 1;
  int repair_end = 2;

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);
  EXPECT_EQ(result.cost, 7);
  VerifyPathIntegrity(result);
}

TEST(InsertShorterRepair, UpDownToStraight) {
  PR full_path;
  full_path.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{1, 1}, 2},
                      {{1, 2}, 3}, {{0, 2}, 4}, {{0, 3}, 5}};
  full_path.actions = {{Action::Up, 1},
                       {Action::Right, 1},
                       {Action::Up, 1},
                       {Action::Left, 1},
                       {Action::Up, 1}};
  full_path.cost = 5;
  full_path.fmin = 5;

  PR repair;
  repair.states = {{{0, 1}, 0}, {{0, 2}, 1}};
  repair.actions = {{Action::Up, 1}};
  repair.cost = 1;
  repair.fmin = 1;

  int repair_start = 1;
  int repair_end = 4;

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);
  EXPECT_EQ(result.cost, 3);
  VerifyPathIntegrity(result);
}

TEST(InsertShorterRepair, UpDownToStraight2) {
  PR full_path;
  full_path.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{0, 2}, 2}, {{1, 2}, 3},
                      {{1, 3}, 4}, {{0, 3}, 5}, {{0, 4}, 6}};
  full_path.actions = {{Action::Up, 1}, {Action::Up, 1},   {Action::Right, 1},
                       {Action::Up, 1}, {Action::Left, 1}, {Action::Up, 1}};
  full_path.cost = 6;
  full_path.fmin = 6;

  PR repair;
  repair.states = {{{0, 2}, 0}, {{0, 3}, 1}};
  repair.actions = {{Action::Up, 1}};
  repair.cost = 1;
  repair.fmin = 1;

  int repair_start = 2;
  int repair_end = 5;

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);
  EXPECT_EQ(result.cost, 4);
  VerifyPathIntegrity(result);
}

using W = Window<NaiveCBSEnvironment<State>, FourConnectedEnvironmentView<NaiveCBSEnvironment<State>,1,1>,1,1>;
NaiveCBSEnvironment<State> env(10, 10, {}, {});
TEST(WindowIntersection, SimpleWindow) {
  PR p1;
  p1.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{0, 2}, 2},
               {{0, 3}, 3}, {{0, 4}, 4}, {{0, 5}, 5}};
  p1.actions = {{Action::Up, 1},
                {Action::Up, 1},
                {Action::Up, 1},
                {Action::Up, 1},
                {Action::Up, 1}};
  p1.cost = 5;
  p1.fmin = 5;
  VerifyPathIntegrity(p1);

  PR p2;
  p2.states = {{{0, 5}, 0}, {{0, 4}, 1}, {{0, 3}, 2},
               {{0, 2}, 3}, {{0, 1}, 4}, {{0, 0}, 5}};
  p2.actions = {{Action::Down, 1},
                {Action::Down, 1},
                {Action::Down, 1},
                {Action::Down, 1},
                {Action::Down, 1}};
  p2.cost = 5;
  p2.fmin = 5;
  VerifyPathIntegrity(p2);

  // Overlaps the window but not involved in indices
  PR p3;
  p3.states = {{{0, 3}, 0}};
  p3.cost = 0;
  p3.fmin = 0;

  W w(State(0, 3), std::vector<size_t>({0, 1}), &env);

  EXPECT_TRUE(w.HasAgent(0));

  EXPECT_EQ(w.env_view_.max_pos_, State(1, 4));
  EXPECT_EQ(w.env_view_.min_pos_, State(-1, 2));

  const auto res = GetWindowStartGoalIndexes<State, Action, W>({p1, p2, p3}, w);
  EXPECT_EQ(res.size(), 2);
  const auto expected1 = std::pair<int, int>(2, 4);
  EXPECT_EQ(res.front(), expected1);
  const auto expected2 = std::pair<int, int>(1, 3);
  EXPECT_EQ(res.back(), expected2);
}

TEST(FourConnectedEnvironmentView, Overlaps) {
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view(
      {0, 0}, {5, 5}, std::vector<State>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view_other(
      {4, 4}, {6, 6}, std::vector<State>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view_three(
      {6, 6}, {9, 9}, std::vector<State>(), &env);

  EXPECT_TRUE(env_view.Overlaps(env_view_other));
  EXPECT_TRUE(env_view_other.Overlaps(env_view));
  EXPECT_TRUE(env_view_other.Overlaps(env_view_three));
  EXPECT_FALSE(env_view.Overlaps(env_view_three));
}

TEST(FourConnectedEnvironmentView, SuccessorOverlaps) {
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view(
      {0, 0}, {2, 2}, std::vector<State>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view_other(
      {4, 4}, {6, 6}, std::vector<State>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view_three(
          {3, 3}, {9, 9}, std::vector<State>(), &env);

  EXPECT_FALSE(env_view.SuccessorOverlaps(env_view_other));
  EXPECT_FALSE(env_view_other.SuccessorOverlaps(env_view));
  EXPECT_TRUE(env_view.SuccessorOverlaps(env_view_three));
}

TEST(FourConnectedEnvironmentView, Merge) {
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view(
      {1, 1}, {2, 2}, std::vector<State>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> env_view_other(
      {0, 4}, {6, 6}, std::vector<State>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<State>> expected(
      {0, 1}, {6, 6}, std::vector<State>(), &env);

  EXPECT_TRUE(expected == env_view.Merge(env_view_other));
}
// test if it goes out of view range, if there is a vertex constraint,
// if there is an edge constraint
// TEST(FourConnectedEnvironmentView, getNeighbors) {
//  TestEnv* env;
//  FourConnectedEnvironmentView<TestEnv> env_view({1,1}, {2,2},
//  std::vector<State>(), env); std::vector<Neighbor<CBSState, CBSAction, int>>
//  neighbors; CBSState s(3,2,2);
//}