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

#include "timer.hpp"
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
using WAMPF = libMultiRobotPlanning::wampf::WAMPF<
    State, IndividualSpaceAction, Cost, Window,
    IndividualSpaceAStar<State, IndividualSpaceAction, Cost,
                         IndividualSpaceEnvironment>,
    NaiveACBSImplementation>;

namespace po = boost::program_options;

std::optional<std::tuple<int, int, std::unordered_set<State>,
                         std::vector<State>, std::vector<State>, std::string>>
ParseInputYAML(int argc, char** argv) {
  po::options_description desc("Allowed options");
  std::string input_file;
  std::string output_file;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&input_file)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&output_file)->required(),
                           "output file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return {};
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return {};
  }

  YAML::Node config = YAML::LoadFile(input_file);

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

  return {{dimx, dimy, obstacles, start_state, goal_state, output_file}};
}

void GenerateOutputYAML(const JointPath* best_path,
                        const std::string& output_file, const double& runtime,
                        const double& time_to_first_sol) {
  int cost = 0;
  int makespan = 0;
  for (const auto& s : *best_path) {
    cost += s.cost;
    makespan = std::max<int>(makespan, s.cost);
  }

  std::ofstream out(output_file);
  out << "statistics:" << std::endl;
  out << "  cost: " << cost << std::endl;
  out << "  makespan: " << makespan << std::endl;
  out << "  runtime: " << runtime << std::endl;
  out << "  time to first solution: " << time_to_first_sol << std::endl;
  out << "schedule:" << std::endl;
  for (size_t a = 0; a < (*best_path).size(); ++a) {
    out << "  agent" << a << ":" << std::endl;
    for (const auto& state : (*best_path)[a].states) {
      out << "    - x: " << state.first.x << std::endl
          << "      y: " << state.first.y << std::endl
          << "      t: " << state.second << std::endl;
    }
  }
}

std::tuple<const JointPath*, double, double> RunWAMPF(WAMPF& wampf) {
  Timer optimal_timer;
  Timer first_sol_timer;
  const JointPath* best_path = nullptr;
  bool should_continue = true;
  bool first_sol = true;
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
  return {best_path, optimal_timer.elapsedSeconds(),
          first_sol_timer.elapsedSeconds()};
}

int main(int argc, char* argv[]) {
  auto parsed_yaml_info = ParseInputYAML(argc, argv);
  if (!parsed_yaml_info) {
    printf("Did not parse YAML\n");
    return 1;
  }
  auto [dimx, dimy, obstacles, start_state, goal_state, output_file] =
      *parsed_yaml_info;

  std::cout << "Starting WAMPF!" << std::endl;
  WAMPF wampf(dimx, dimy, obstacles, start_state, goal_state);

  auto [best_path, runtime, time_to_first_sol] = RunWAMPF(wampf);

  if (best_path == nullptr) {
    std::cout << "Planning NOT successful!" << std::endl;
    return 1;
  }
  std::cout << "Planning successful! " << std::endl;

  GenerateOutputYAML(best_path, output_file, runtime, time_to_first_sol);
}