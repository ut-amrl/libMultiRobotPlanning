#pragma once

#include <boost/functional/hash.hpp>

namespace libMultiRobotPlanning {

struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};
}  // namespace libMultiRobotPlanning

namespace std {
template <>
struct hash<libMultiRobotPlanning::State> {
  size_t operator()(const libMultiRobotPlanning::State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

namespace libMultiRobotPlanning {

struct Location {
  Location(int x, int y) : x(x), y(y) {}

  Location(const Location&) = default;
  Location(Location&&) = default;
  Location& operator=(const Location&) = default;
  Location& operator=(Location&&) = default;

  bool StateEquals(const State& s) const { return x == s.x && y == s.y; }

  bool operator==(const Location& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const Location& s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  int x;
  int y;
};
}  // namespace libMultiRobotPlanning

namespace std {
template <>
struct hash<libMultiRobotPlanning::Location> {
  size_t operator()(const libMultiRobotPlanning::Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std
