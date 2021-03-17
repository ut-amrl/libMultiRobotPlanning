#pragma once

#include <algorithm>
#include <libMultiRobotPlanning/cbs.hpp>
#include <vector>

#include "wampf_state.h"

#include "four_grid_env_view.h"
#include "wampf_naive_cbs_env.h"
using State = libMultiRobotPlanning::State;
using Action = libMultiRobotPlanning::IndividualSpaceAction;
using Location = libMultiRobotPlanning::Location;

namespace libMultiRobotPlanning {

template <typename Env, typename EnvView, int kStartRadius = 2,
          int kRadiusGrowth = 1>
struct Window {
 private:
  Window() = delete;

 public:
  std::vector<size_t> agent_idxs_;
  EnvView env_view_;
  CBS<State, Action, int, naive_cbs_wampf_impl::Conflict,
      naive_cbs_wampf_impl::Constraints, EnvView>
      cbs_;

  Window(Location location, const std::vector<size_t> agent_idxs,
         const Env* env)
      : agent_idxs_(agent_idxs),
        env_view_(location, std::vector<Location>(), env),
        cbs_(&env_view_) {}

  Window(const std::vector<size_t> agent_idxs, EnvView view)
      : agent_idxs_(agent_idxs), env_view_(std::move(view)), cbs_(&env_view_) {}

  Window(Location min_pos, Location max_pos,
         const std::vector<size_t> agent_idxs, const Env* env)
      : agent_idxs_(agent_idxs),
        env_view_(min_pos, max_pos, std::vector<Location>(), env),
        cbs_(&env_view_) {}

  Window(const Window&) = delete;
  Window(Window&&) = delete;

  Window& operator=(const Window&) = delete;
  Window& operator=(Window&&) = delete;

  bool operator==(const Window& other) const {
    return (env_view_ == other.env_view_) && (agent_idxs_ == other.agent_idxs_);
  }

  friend std::ostream& operator<<(std::ostream& os, const Window& c) {
    os << "agent_idxs_: [";
    for (const auto& a : c.agent_idxs_) {
      os << a << ", ";
    }
    os << "] " << c.env_view_;
    return os;
  }

  bool operator!=(const Window& other) const { return !(*this == other); }

  bool HasAgent(const size_t agent_idx) const {
    return (std::find(agent_idxs_.begin(), agent_idxs_.end(), agent_idx) !=
            agent_idxs_.end());
  }

  bool Contains(const State& s) const { return env_view_.Contains(s); }

  bool Contains(const Location& s) const { return env_view_.Contains(s); }

  bool OverlappingAgents(const Window& other) const {
    for (const auto& a : other.agent_idxs_) {
      if (HasAgent(a)) {
        return true;
      }
    }
    return false;
  }

  bool Overlaps(const Window& other) const {
    if (!OverlappingAgents(other)) {
      return false;
    }

    return env_view_.Overlaps(other.env_view_);
  }

  bool SuccessorOverlaps(const Window& other) const {
    if (!OverlappingAgents(other)) {
      return false;
    }

    return env_view_.SuccessorOverlaps(other.env_view_);
  }

  void Grow() { env_view_.Grow(); }

  bool ShouldQuit() const { return env_view_.ShouldQuit(); }

  std::unique_ptr<Window> Merge(const Window& o) const {
    auto joined_agent_idxs = agent_idxs_;
    joined_agent_idxs.insert(joined_agent_idxs.end(), o.agent_idxs_.begin(),
                             o.agent_idxs_.end());
    std::sort(joined_agent_idxs.begin(), joined_agent_idxs.end());
    const auto it =
        std::unique(joined_agent_idxs.begin(), joined_agent_idxs.end());
    joined_agent_idxs.resize(std::distance(joined_agent_idxs.begin(), it));
    std::sort(joined_agent_idxs.begin(), joined_agent_idxs.end());

    return std::make_unique<Window>(joined_agent_idxs,
                                    env_view_.Merge(o.env_view_));
  }
};

}  // namespace libMultiRobotPlanning