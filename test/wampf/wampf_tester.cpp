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
#include "../../include/libMultiRobotPlanning/utils.hpp"

using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;
using libMultiRobotPlanning::Window;
using libMultiRobotPlanning::wampf::GetWindowStartGoalIndexes;
using libMultiRobotPlanning::wampf::InsertPathRepair;
using PR = PlanResult<State, IndividualSpaceAction, int>;
using Action = IndividualSpaceAction;
using Action = libMultiRobotPlanning::IndividualSpaceAction;
using State = libMultiRobotPlanning::State;
using Location = libMultiRobotPlanning::Location;
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

void PadPlanResult(PlanResult<State, Action, int>& path,
                   int required_len) {
  while (static_cast<int>(path.states.size()) < required_len) {
    auto back_state_copy = path.states.back();
    path.states.emplace_back(back_state_copy);
    path.actions.push_back({Action::Wait, 1});
    path.cost += 1;
  }
}

void FixStateTimes(PlanResult<State, Action, int>& path, int start_idx) {
  if (start_idx != 0) {
    path.states[start_idx].first.time = path.states[start_idx - 1].first.time + 1;
  }
  for (int i = start_idx + 1; i < path.states.size(); i++) {
    path.states[i].first.time = path.states[i - 1].first.time;
  }
}

TEST(InsertLongerRepair, StraightToUpDown) {
  PR full_path;
  full_path.states = {{{0, 0, 0}, 0}, {{1, 0, 1}, 1}, {{2, 0, 2}, 2},
                      {{3, 0, 3}, 3}, {{4, 0, 4}, 4}, {{5, 0, 5}, 5}};
  full_path.actions = {{Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1}};
  full_path.cost = 5;
  full_path.fmin = 5;

  PR repair;
  repair.states = {{{1, 0, 1}, 0}, {{2, 1, 1}, 1}, {{3, 1, 2}, 2}, {{4, 0, 2}, 3}};
  repair.actions = {{Action::Right, 1}, {Action::Up, 1}, {Action::Left, 1}};
  repair.cost = 3;
  repair.fmin = 3;

  int repair_start = 1;
  int repair_end = 2;

  PadPlanResult(repair, repair_end - repair_start);

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);

  FixStateTimes(full_path, repair_start);
  EXPECT_EQ(result.cost, 7);
  VerifyPathIntegrity(result);
}

TEST(InsertShorterRepair, UpDownToStraight) {
  PR full_path;
  full_path.states = {{{0, 0, 0}, 0}, {{1, 0, 1}, 1}, {{2, 1, 1}, 2},
                      {{3, 1, 2}, 3}, {{4, 0, 2}, 4}, {{5, 0, 3}, 5}};
  full_path.actions = {{Action::Up, 1},
                       {Action::Right, 1},
                       {Action::Up, 1},
                       {Action::Left, 1},
                       {Action::Up, 1}};
  full_path.cost = 5;
  full_path.fmin = 5;

  PR repair;
  repair.states = {{{1, 0, 1}, 0}, {{2, 0, 2}, 1}};
  repair.actions = {{Action::Up, 1}};
  repair.cost = 1;
  repair.fmin = 1;

  int repair_start = 1;
  int repair_end = 4;

  PadPlanResult(repair, repair_end - repair_start);

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);

  FixStateTimes(full_path, repair_start);
  EXPECT_EQ(result.cost, 3);
  VerifyPathIntegrity(result);
}

TEST(InsertShorterRepair, UpDownToStraight2) {
  PR full_path;
  full_path.states = {{{0, 0, 0}, 0}, {{1, 0, 1}, 1}, {{2, 0, 2}, 2}, {{3, 1, 2}, 3},
                      {{4, 1, 3}, 4}, {{5, 0, 3}, 5}, {{6, 0, 4}, 6}};
  full_path.actions = {{Action::Up, 1}, {Action::Up, 1},   {Action::Right, 1},
                       {Action::Up, 1}, {Action::Left, 1}, {Action::Up, 1}};
  full_path.cost = 6;
  full_path.fmin = 6;

  PR repair;
  repair.states = {{{0, 0, 2}, 0}, {{1, 0, 3}, 1}};
  repair.actions = {{Action::Up, 1}};
  repair.cost = 1;
  repair.fmin = 1;

  int repair_start = 2;
  int repair_end = 5;
  PadPlanResult(repair, repair_end - repair_start);

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);

  FixStateTimes(full_path, repair_start);
  EXPECT_EQ(result.cost, 4);
  VerifyPathIntegrity(result);
}

using W = Window<NaiveCBSEnvironment<Location>,
                 FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>, 1, 1>,
                 1, 1>;
NaiveCBSEnvironment<Location> env(10, 11, {{6, 7}, {6, 9}}, {{5, 10}});
TEST(WindowIntersection, SimpleWindow) {
  PR p1;
  p1.states = {{{0, 0, 0}, 0}, {{1, 0, 1}, 1}, {{2, 0, 2}, 2},
               {{3, 0, 3}, 3}, {{4, 0, 4}, 4}, {{5, 0, 5}, 5}};
  p1.actions = {{Action::Up, 1},
                {Action::Up, 1},
                {Action::Up, 1},
                {Action::Up, 1},
                {Action::Up, 1}};
  p1.cost = 5;
  p1.fmin = 5;
  VerifyPathIntegrity(p1);

  PR p2;
  p2.states = {{{0, 0, 5}, 0}, {{1, 0, 4}, 1}, {{2, 0, 3}, 2},
               {{3, 0, 2}, 3}, {{4, 0, 1}, 4}, {{5, 0, 0}, 5}};
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
  p3.states = {{{0, 0, 3}, 0}};
  p3.cost = 0;
  p3.fmin = 0;

  W w(Location(0, 3), std::vector<size_t>({0, 1}), &env);

  EXPECT_TRUE(w.HasAgent(0));

  EXPECT_EQ(w.env_view_.max_pos_, Location(1, 4));
  EXPECT_EQ(w.env_view_.min_pos_, Location(-1, 2));

  const auto res = GetWindowStartGoalIndexes<State, Action, W>({p1, p2, p3}, w);
  EXPECT_EQ(res.size(), 2);
  const auto expected1 = std::pair<int, int>(2, 4);
  EXPECT_EQ(res.front(), expected1);
  const auto expected2 = std::pair<int, int>(1, 3);
  EXPECT_EQ(res.back(), expected2);
}

TEST(FourConnectedEnvironmentView, Overlaps) {
  {
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {0, 0}, {5, 5}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {4, 4}, {6, 6}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_three(
        {6, 6}, {9, 9}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
    EXPECT_TRUE(env_view_other.Overlaps(env_view_three));
    EXPECT_FALSE(env_view.Overlaps(env_view_three));
  }
  {
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {2, 0}, {4, 8}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 8}, {3, 9}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_three(
        {3, 1}, {6, 7}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
    EXPECT_TRUE(env_view.Overlaps(env_view_three));
    EXPECT_FALSE(env_view_other.Overlaps(env_view_three));
  }
  {
    // on top of each other
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {1, 3}, {9, 6}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 3}, {9, 6}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // one eclipses the other
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {3, 1}, {6, 9}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {2, 2}, {5, 8}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // plus sign
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {4, 1}, {5, 9}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 3}, {9, 4}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // t shaped
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {3, 2}, {4, 8}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {2, 7}, {7, 9}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
}

TEST(FourConnectedEnvironmentView, SuccessorOverlaps) {
  {
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {0, 0}, {2, 2}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {4, 4}, {6, 6}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_three(
        {3, 3}, {9, 9}, std::vector<Location>(), &env);

    EXPECT_FALSE(env_view.SuccessorOverlaps(env_view_other));
    EXPECT_FALSE(env_view_other.SuccessorOverlaps(env_view));
    EXPECT_TRUE(env_view.SuccessorOverlaps(env_view_three));
  }
  {
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {1, 8}, {3, 9}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {3, 1}, {6, 7}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.SuccessorOverlaps(env_view_other));
    EXPECT_TRUE(env_view_other.SuccessorOverlaps(env_view));
  }
  {
    // on top of each other
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {1, 3}, {9, 6}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 3}, {9, 6}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // one eclipses the other
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {3, 1}, {6, 9}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {2, 2}, {5, 8}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // plus sign
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {4, 1}, {5, 9}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 3}, {9, 4}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // growing into plus from t-shaped
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {1, 3}, {7, 8}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {4, 1}, {5, 8}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // growing into exact same window
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {2, 4}, {6, 7}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 3}, {7, 8}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
  {
    // growing from exact same to eclipsing
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
        {1, 3}, {7, 8}, std::vector<Location>(), &env);
    FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
        {1, 3}, {7, 8}, std::vector<Location>(), &env);

    EXPECT_TRUE(env_view.Overlaps(env_view_other));
    EXPECT_TRUE(env_view_other.Overlaps(env_view));
  }
}

TEST(FourConnectedEnvironmentView, Merge) {
  FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
      {1, 1}, {2, 2}, std::vector<Location>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view_other(
      {0, 4}, {6, 6}, std::vector<Location>(), &env);
  FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> expected(
      {0, 1}, {6, 6}, std::vector<Location>(), &env);

  EXPECT_TRUE(expected == env_view.Merge(env_view_other));
}

using N = Neighbor<State, Action, int>;
TEST(FourConnectedEnvironmentView, getNeighbors) {
  FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view(
      {3, 5}, {6, 12}, {{5, 10}}, &env);
  std::vector<N> neighbors;
  Constraints constraints;

  // for State(2,3,10) only neighbor should be State(3,4,10) and
  // State(3,3,10)
  constraints.vertexConstraints.insert({3, 3, 9});

  // for State(2,6,5), only neighbor is State(3,5,5)
  constraints.vertexConstraints.insert({3, 6, 5});
  constraints.edgeConstraints.insert({2, 6, 5, 6, 6});

  // for State(2,6,8), 2 neighbors: State(3,6,8) and State(3,5,8)
  // because of obstacles in env
  constraints.edgeConstraints.insert({2, 5, 8, 4, 8});

  env_view.setLowLevelContext(0, &constraints);

  // tests vertex constraints, environment bounds, and environment view bounds
  env_view.getNeighbors({2, 3, 10}, neighbors);
  NP_CHECK_EQ(neighbors.size(), 2);
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 4, 10), Action::Right, 1)));
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 3, 10), Action::Wait, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 3, 9), Action::Down, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 3, 11), Action::Up, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 2, 10), Action::Left, 1)));

  // tests vertex constraints, edge constraints, and environment view bounds
  env_view.getNeighbors({2, 6, 5}, neighbors);
  NP_CHECK_EQ(neighbors.size(), 1);
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 5, 5), Action::Left, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 6, 5), Action::Wait, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 7, 5), Action::Right, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 6, 4), Action::Down, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 6, 6), Action::Up, 1)));

  // tests functionality without any constraints
  env_view.getNeighbors({2, 4, 7}, neighbors);
  NP_CHECK_EQ(neighbors.size(), 5);
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 4, 8), Action::Up, 1)));
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 4, 6), Action::Down, 1)));
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 4, 7), Action::Wait, 1)));
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 3, 7), Action::Left, 1)));
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 5, 7), Action::Right, 1)));

  // tests obstacles and environment view bounds
  env_view.getNeighbors({2, 6, 8}, neighbors);
  NP_CHECK_EQ(neighbors.size(), 2);
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 6, 8), Action::Wait, 1)));
  EXPECT_TRUE(std::count(neighbors.begin(), neighbors.end(),
                         N(State(3, 5, 8), Action::Left, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 7, 8), Action::Right, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 6, 7), Action::Down, 1)));
  EXPECT_FALSE(std::count(neighbors.begin(), neighbors.end(),
                          N(State(3, 6, 9), Action::Up, 1)));
}

// fourconnected grow
TEST(FourConnectedEnvironmentView, Grow) {
  FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>> env_view({6, 3}, {},
                                                                    &env);
  EXPECT_TRUE(env_view.min_pos_ == Location(4, 1));
  EXPECT_TRUE(env_view.max_pos_ == Location(8, 5));

  env_view.Grow();
  EXPECT_TRUE(env_view.min_pos_ == Location(3, 0));
  EXPECT_TRUE(env_view.max_pos_ == Location(9, 6));
}

using WDefault =
    Window<NaiveCBSEnvironment<Location>,
           FourConnectedEnvironmentView<NaiveCBSEnvironment<Location>>>;
TEST(Window, Merge) {
  WDefault w({4, 6}, {2, 3, 6}, &env);
  WDefault w_other({5, 1}, {9, 6}, {1, 3, 6, 8}, &env);
  WDefault w_result({2, 1}, {9, 8}, {1, 2, 3, 6, 8}, &env);

  EXPECT_TRUE(*w.Merge(w_other) == w_result);
  EXPECT_TRUE(*w.Merge(w_other) == *w_other.Merge(w));
}

TEST(Window, ShouldQuit) {
  {
    WDefault w({0, 0}, {11, 11}, {1, 2, 3}, &env);
    EXPECT_TRUE(w.ShouldQuit());
  }
  {
    WDefault w({3, 8}, {13, 14}, {1, 2, 3}, &env);
    EXPECT_FALSE(w.ShouldQuit());
  }
  {
    WDefault w({-2, -3}, {13, 14}, {1, 2, 3}, &env);
    EXPECT_TRUE(w.ShouldQuit());
  }
  {
    WDefault w({3, 8}, {4, 9}, {1, 2, 3}, &env);
    EXPECT_FALSE(w.ShouldQuit());
  }
}