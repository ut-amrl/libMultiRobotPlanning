#pragma once

#include <boost/functional/hash.hpp>

namespace libMultiRobotPlanning {

struct State {
  State() = delete;
  State(int x, int y) : x(x), y(y) {}

  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ") ";
  }

  int x;
  int y;
};
}  // namespace libMultiRobotPlanning

namespace std {
template <>
struct hash<libMultiRobotPlanning::State> {
  size_t operator()(const libMultiRobotPlanning::State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std