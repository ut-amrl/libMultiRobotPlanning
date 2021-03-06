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
using naive_cbs_wampf_impl::CBSAction;
using naive_cbs_wampf_impl::CBSState;
using naive_cbs_wampf_impl::Conflict;
using naive_cbs_wampf_impl::Constraints;
using naive_cbs_wampf_impl::FourConnectedEnvironmentView;
using naive_cbs_wampf_impl::NaiveCBSEnvironment;

TEST(NaiveCBSEnvironment, GetFirstConflict) {

}