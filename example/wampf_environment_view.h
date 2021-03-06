#pragma once

#include <unordered_set>

#include <boost/functional/hash.hpp>

#include <libMultiRobotPlanning/cbs.hpp>
#include <libMultiRobotPlanning/neighbor.hpp>
#include <libMultiRobotPlanning/planresult.hpp>
#include "wampf_naive_cbs_env.h"
// should be abstract
namespace naive_cbs_wampf_impl {
template <typename Environment, typename State, typename Location, int kStartRadius = 2,
          int kRadiusGrowth = 1, typename Action>
class EnvironmentView {
 public:
  virtual void UpdateGoals(std::vector<Location> goals) = 0;

  virtual int admissibleHeuristic(const State& s) const = 0;

  virtual bool ShouldQuit() const = 0;

  virtual bool Contains(const Location& s) const = 0;

  virtual void Grow() = 0;

  //    virtual bool SuccessorOverlaps(const EnvironmentView& other) const;
  //
  //    virtual bool Overlaps(const EnvironmentView& other) const;

  virtual void setLowLevelContext(size_t agentIdx,
                                  const Constraints* constraints) = 0;

  virtual bool isSolution(const State& s) const = 0;

  virtual void getNeighbors(
      const State& s,
      std::vector<Neighbor<State, Action, int>>& neighbors) const = 0;

  virtual bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int>>& solution,
      Conflict& result) const = 0;

  virtual void createConstraintsFromConflict(
      const Conflict& conflict,
      std::map<size_t, Constraints>& constraints) const = 0;

  virtual void onExpandHighLevelNode(int /*cost*/) = 0;

  virtual void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                                    int /*gScore*/) = 0;
};

}  // namespace naive_cbs_wampf_impl