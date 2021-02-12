#pragma once

#include <algorithm>
#include <vector>

#include "wampf_state.h"

#include "wampf_naive_cbs_env.h"
namespace libMultiRobotPlanning {

template <typename Env, typename EnvView, int kStartRadius = 2,
          int kRadiusGrowth = 1>
struct Window {
 private:
  Window() = delete;

 public:
  std::vector<size_t> agent_idxs_;
  EnvView env_view_;
  CBS<naive_cbs_wampf_impl::CBSState, naive_cbs_wampf_impl::CBSAction, int,
      naive_cbs_wampf_impl::Conflict, naive_cbs_wampf_impl::Constraints,
      EnvView>
      cbs_;

  //  Window& operator=(const Window& other) {
  //    this->min_pos_ = other.min_pos_;
  //    this->max_pos_ = other.max_pos_;
  //    this->
  //  }

  Window(State state, const std::vector<size_t> agent_idxs, const Env* env)
      : agent_idxs_(agent_idxs),
        env_view_(state, state, std::vector<State>(), env),
        cbs_(&env_view_) {
    env_view_.min_pos_.x -= kStartRadius;
    env_view_.min_pos_.y -= kStartRadius;
    env_view_.max_pos_.x += kStartRadius;
    env_view_.max_pos_.y += kStartRadius;
  }

  Window(State min_pos, State max_pos, const std::vector<size_t> agent_idxs,
         const Env* env)
      : agent_idxs_(agent_idxs),
        env_view_(min_pos, max_pos, std::vector<State>(), env),
        cbs_(&env_view_) {}

  Window(const Window&) = default;
  Window(Window&&) = default;

  Window& operator=(const Window&) = default;
  Window& operator=(Window&&) = default;

  bool operator==(const Window& other) const {
    return (env_view_.min_pos_ == other.env_view_.min_pos_) &&
           (env_view_.max_pos_ == other.env_view_.max_pos_) &&
           (agent_idxs_ == other.agent_idxs_);
  }

  bool operator!=(const Window& other) const { return !(*this == other); }

  bool HasAgent(const size_t agent_idx) const {
    return (std::find(agent_idxs_.begin(), agent_idxs_.end(), agent_idx) !=
            agent_idxs_.end());
  }

  bool Contains(const State& s) const {
    return ((s.x >= env_view_.min_pos_.x && s.x <= env_view_.max_pos_.x) &&
            (s.y >= env_view_.min_pos_.y && s.y <= env_view_.max_pos_.y));
  }

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

    State off1(env_view_.min_pos_.x, env_view_.max_pos_.y);
    State off2(env_view_.max_pos_.x, env_view_.min_pos_.y);

    // Check if our four corners are inside their box.
    if (other.Contains(env_view_.min_pos_) ||
        other.Contains(env_view_.max_pos_) || other.Contains(off1) ||
        other.Contains(off2)) {
      return true;
    }

    State other_off1(other.env_view_.min_pos_.x, other.env_view_.max_pos_.y);
    State other_off2(other.env_view_.max_pos_.x, other.env_view_.min_pos_.y);

    // Check if their four corners are inside our box.
    if (Contains(other.env_view_.min_pos_) ||
        Contains(other.env_view_.max_pos_) || Contains(other_off1) ||
        Contains(other_off2)) {
      return true;
    }
    return false;
  }

  bool SuccessorOverlaps(const Window& other) const {
    if (!OverlappingAgents(other)) {
      return false;
    }

    State min_growth(env_view_.min_pos_.x - kRadiusGrowth,
                     env_view_.min_pos_.y - kRadiusGrowth);
    State max_growth(env_view_.max_pos_.x + kRadiusGrowth,
                     env_view_.max_pos_.y + kRadiusGrowth);
    State off1(env_view_.min_pos_.x - kRadiusGrowth,
               env_view_.max_pos_.y + kRadiusGrowth);
    State off2(env_view_.max_pos_.x + kRadiusGrowth,
               env_view_.min_pos_.y - kRadiusGrowth);

    // Check if our four corners are inside their box.
    if (other.Contains(min_growth) || other.Contains(max_growth) ||
        other.Contains(off1) || other.Contains(off2)) {
      return true;
    }

    // Inflating their box is equivalent to inflating our box.
    State other_off1(other.env_view_.min_pos_.x - kRadiusGrowth,
                     other.env_view_.max_pos_.y + kRadiusGrowth);
    State other_off2(other.env_view_.max_pos_.x + kRadiusGrowth,
                     other.env_view_.min_pos_.y - kRadiusGrowth);

    // Check if their four corners are inside our box.
    if (Contains(other.env_view_.min_pos_) ||
        Contains(other.env_view_.max_pos_) || Contains(other_off1) ||
        Contains(other_off2)) {
      return true;
    }
    return false;
  }

  void Grow() {
    env_view_.min_pos_.x -= kRadiusGrowth;
    env_view_.min_pos_.y -= kRadiusGrowth;
    env_view_.max_pos_.x += kRadiusGrowth;
    env_view_.max_pos_.y += kRadiusGrowth;
  }

  bool ShouldQuit() const { return false; }

  std::unique_ptr<Window> Merge(const Window& o) const {
    int min_x = std::min(env_view_.min_pos_.x, o.env_view_.min_pos_.x);
    int max_x = std::max(env_view_.max_pos_.x, o.env_view_.max_pos_.x);
    int min_y = std::min(env_view_.min_pos_.y, o.env_view_.min_pos_.y);
    int max_y = std::max(env_view_.max_pos_.y, o.env_view_.max_pos_.y);

    auto joined_agent_idxs = agent_idxs_;
    joined_agent_idxs.insert(joined_agent_idxs.end(), o.agent_idxs_.begin(),
                             o.agent_idxs_.end());
    std::sort(joined_agent_idxs.begin(), joined_agent_idxs.end());
    const auto it =
        std::unique(joined_agent_idxs.begin(), joined_agent_idxs.end());
    joined_agent_idxs.resize(std::distance(joined_agent_idxs.begin(), it));
    std::sort(joined_agent_idxs.begin(), joined_agent_idxs.end());

    std::unique_ptr<Window> new_w =
        std::make_unique<Window>(State(min_x, min_y), State(max_x, max_y),
                                 joined_agent_idxs, env_view_.getEnvPtr());
    return new_w;
  }
};

}  // namespace libMultiRobotPlanning