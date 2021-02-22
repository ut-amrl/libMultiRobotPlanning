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

#include <yaml-cpp/yaml.h>

#include "wampf_individual.h"
#include "wampf_naive_cbs.h"
#include "wampf_naive_cbs_env.h"
#include "wampf_state.h"
#include "wampf_window.h"
#include "timer.hpp"

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

int main(int argc, char* argv[]) {
  using WAMPF = libMultiRobotPlanning::wampf::WAMPF<
      State, IndividualSpaceAction, Cost, Window,
      IndividualSpaceAStar<State, IndividualSpaceAction, Cost,
                           IndividualSpaceEnvironment>,
      NaiveACBSImplementation>;

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
          "input,i", po::value<std::string>(&inputFile)->required(),
          "input file (YAML)")("output,o",
                               po::value<std::string>(&outputFile)->required(),
                               "output file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<State> obstacles;
  JointState goal_state;
  JointState start_state;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(State(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    start_state.emplace_back(State(start[0].as<int>(), start[1].as<int>()));
    goal_state.emplace_back(State(goal[0].as<int>(), goal[1].as<int>()));
  }


  std::cout << "Starting WAMPF!" << std::endl;
  WAMPF wampf(dimx, dimy, obstacles, start_state, goal_state);
  const JointPath* best_path = nullptr;
  bool should_continue = true;
  bool first_sol = true;
  Timer optimal_timer, first_sol_timer;
  while (should_continue) {
    auto [pi, numer, denom] = wampf.RecWAMPF();
    should_continue = !(numer == denom);
    best_path = pi;
    if (first_sol) {
      first_sol_timer.stop();
      first_sol = false;
    }
  }
  optimal_timer.stop();
  if (best_path != nullptr) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : *best_path) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << optimal_timer.elapsedSeconds() << std::endl;
    out << "  first solution runtime: " << first_sol_timer.elapsedSeconds() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < (*best_path).size(); ++a) {
      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : (*best_path)[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

}