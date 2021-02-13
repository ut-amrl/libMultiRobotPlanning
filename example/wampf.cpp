#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_set>

#include <libMultiRobotPlanning/individual_space_astar.hpp>
#include <libMultiRobotPlanning/neighbor.hpp>
#include <libMultiRobotPlanning/wampf.hpp>
#include <libMultiRobotPlanning/wampf_utils.hpp>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include "wampf_individual.h"
#include "wampf_naive_cbs.h"
#include "wampf_naive_cbs_env.h"
#include "wampf_state.h"
#include "wampf_window.h"

using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::IndividualSpaceAStar;
using libMultiRobotPlanning::IndividualSpaceEnvironment;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;
using wampf_impl::NaiveACBSImplementation;

using Env = naive_cbs_wampf_impl::NaiveCBSEnvironment<State>;
using EnvView = naive_cbs_wampf_impl::FourConnectedEnvironmentView<
    naive_cbs_wampf_impl::NaiveCBSEnvironment<State>>;

using Window = libMultiRobotPlanning::Window<Env, EnvView>;

using Cost = int;
using JointState = std::vector<State>;
using JointPath = std::vector<PlanResult<State, IndividualSpaceAction, Cost>>;

int main() {
  using WAMPF = libMultiRobotPlanning::wampf::WAMPF<
      State, IndividualSpaceAction, Cost, Window,
      IndividualSpaceAStar<State, IndividualSpaceAction, Cost,
                           IndividualSpaceEnvironment>,
      NaiveACBSImplementation>;

  std::cout << "Starting WAMPF!" << std::endl;
  JointState start_state = {{30, 50}, {70, 50}};
  JointState goal_state = {{70, 50}, {30, 50}};
  WAMPF wampf(100, 100, {{32, 50}, {32, 49}, {32, 51}}, start_state,
              goal_state);
  wampf.RecWAMPF();
  const auto& path = wampf.GetPath();
  for (const auto& p : path) {
    std::cout << p << std::endl;
  }
}