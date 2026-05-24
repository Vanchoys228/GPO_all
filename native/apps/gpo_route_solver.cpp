#include "tsp/algorithms.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kMetricScale = 1000;
constexpr int kPenaltyDistance = 100000000;

enum class TaskKind {
  kTsp,
  kHamiltonianChain,
  kShortestRoute,
};

struct InputPoint {
  double x = 0.0;
  double y = 0.0;
};

struct SolveConfig {
  std::string algorithm_key;
  TaskKind task = TaskKind::kTsp;
  std::uint32_t seed = 1337U;
  tsp::AlgorithmParams params;
  std::vector<InputPoint> points;
};

struct SolveOutput {
  bool closed = false;
  double length = 0.0;
  std::vector<int> order;
  std::vector<InputPoint> route;
};

std::string trim(const std::string& value) {
  std::size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) ++start;

  std::size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) --end;
  return value.substr(start, end - start);
}

TaskKind parse_task(const std::string& value) {
  if (value == "tsp") return TaskKind::kTsp;
  if (value == "hamiltonian_chain") return TaskKind::kHamiltonianChain;
  if (value == "shortest_route") return TaskKind::kShortestRoute;
  throw std::runtime_error("Unknown task key: " + value);
}

int scaled_distance(const InputPoint& left, const InputPoint& right) {
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  return static_cast<int>(std::llround(std::hypot(dx, dy) * static_cast<double>(kMetricScale)));
}

double route_length(const std::vector<InputPoint>& route, bool closed) {
  if (route.size() <= 1) return 0.0;

  double total = 0.0;
  const std::size_t edge_count = closed ? route.size() : route.size() - 1;
  for (std::size_t index = 0; index < edge_count; ++index) {
    const std::size_t next = (index + 1) % route.size();
    const double dx = route[index].x - route[next].x;
    const double dy = route[index].y - route[next].y;
    total += std::hypot(dx, dy);
  }
  return total;
}

std::vector<int> extract_open_order_from_augmented_cycle(
    const std::vector<int>& cycle_order,
    int dummy_index) {
  const auto it = std::find(cycle_order.begin(), cycle_order.end(), dummy_index);
  if (it == cycle_order.end()) throw std::runtime_error("Dummy node not found in solver output.");

  const int dummy_pos = static_cast<int>(std::distance(cycle_order.begin(), it));
  std::vector<int> order;
  order.reserve(cycle_order.size() - 1);
  for (int offset = 1; offset < static_cast<int>(cycle_order.size()); ++offset) {
    const int value = cycle_order[(dummy_pos + offset) % static_cast<int>(cycle_order.size())];
    if (value != dummy_index) order.push_back(value);
  }
  return order;
}

std::vector<int> orient_fixed_endpoints(std::vector<int> order, int start_node, int end_node) {
  if (order.empty()) return order;
  if (order.front() == start_node && order.back() == end_node) return order;

  std::reverse(order.begin(), order.end());
  if (order.front() == start_node && order.back() == end_node) return order;

  throw std::runtime_error("Unable to orient fixed-endpoint route.");
}

tsp::Problem build_euclidean_problem(const std::vector<InputPoint>& points) {
  tsp::Problem problem;
  problem.name = "gpo";
  problem.cities.reserve(points.size());
  for (int index = 0; index < static_cast<int>(points.size()); ++index) {
    problem.cities.push_back(tsp::City{index, points[index].x, points[index].y});
  }

  problem.distance_matrix.assign(points.size(), std::vector<int>(points.size(), 0));
  for (int left = 0; left < static_cast<int>(points.size()); ++left) {
    for (int right = left + 1; right < static_cast<int>(points.size()); ++right) {
      const int distance = scaled_distance(points[left], points[right]);
      problem.distance_matrix[left][right] = distance;
      problem.distance_matrix[right][left] = distance;
    }
  }
  return problem;
}

tsp::Problem build_augmented_problem(
    const std::vector<InputPoint>& points,
    TaskKind task,
    int* dummy_index_out) {
  tsp::Problem problem = build_euclidean_problem(points);
  const int dummy_index = static_cast<int>(problem.cities.size());
  problem.cities.push_back(tsp::City{dummy_index, 0.0, 0.0});

  for (std::vector<int>& row : problem.distance_matrix) row.push_back(kPenaltyDistance);
  problem.distance_matrix.push_back(std::vector<int>(dummy_index + 1, kPenaltyDistance));
  problem.distance_matrix[dummy_index][dummy_index] = 0;

  if (task == TaskKind::kHamiltonianChain) {
    for (int node = 0; node < dummy_index; ++node) {
      problem.distance_matrix[dummy_index][node] = 0;
      problem.distance_matrix[node][dummy_index] = 0;
    }
  } else if (task == TaskKind::kShortestRoute) {
    if (dummy_index < 2) {
      throw std::runtime_error("Shortest-route task requires at least two points.");
    }
    problem.distance_matrix[dummy_index][0] = 0;
    problem.distance_matrix[0][dummy_index] = 0;
    problem.distance_matrix[dummy_index][dummy_index - 1] = 0;
    problem.distance_matrix[dummy_index - 1][dummy_index] = 0;
  }

  *dummy_index_out = dummy_index;
  return problem;
}

tsp::AlgorithmRun run_algorithm(
    const tsp::Problem& problem,
    const std::string& algorithm_key,
    const tsp::AlgorithmParams& params,
    std::uint32_t seed) {
  if (algorithm_key == "ga_tabu") return tsp::run_ga_tabu(problem, params, seed);
  if (algorithm_key == "otshig") return tsp::run_annealing(problem, params, seed);
  if (algorithm_key == "rasseivanie") return tsp::run_scatter_search(problem, params, seed);
  if (algorithm_key == "cuckoo") return tsp::run_cuckoo_search(problem, params, seed);
  throw std::runtime_error("Unknown algorithm key: " + algorithm_key);
}

SolveOutput solve_route(const SolveConfig& config) {
  if (config.points.empty()) return SolveOutput{};
  if (config.points.size() == 1) {
    return SolveOutput{
        config.task == TaskKind::kTsp,
        0.0,
        {0},
        {config.points[0]},
    };
  }

  std::vector<int> order;
  bool closed = false;

  if (config.task == TaskKind::kTsp) {
    const tsp::Problem problem = build_euclidean_problem(config.points);
    const tsp::AlgorithmRun run =
        run_algorithm(problem, config.algorithm_key, config.params, config.seed);
    order = run.best.order;
    closed = true;
  } else {
    int dummy_index = -1;
    const tsp::Problem problem = build_augmented_problem(config.points, config.task, &dummy_index);
    const tsp::AlgorithmRun run =
        run_algorithm(problem, config.algorithm_key, config.params, config.seed);
    order = extract_open_order_from_augmented_cycle(run.best.order, dummy_index);
    if (config.task == TaskKind::kShortestRoute) {
      order = orient_fixed_endpoints(std::move(order), 0, static_cast<int>(config.points.size()) - 1);
    }
  }

  std::vector<InputPoint> route;
  route.reserve(order.size() + (closed ? 1U : 0U));
  for (int index : order) {
    if (index < 0 || index >= static_cast<int>(config.points.size())) {
      throw std::runtime_error("Solver returned an invalid point index.");
    }
    route.push_back(config.points[index]);
  }
  if (closed && !route.empty()) route.push_back(route.front());

  return SolveOutput{
      closed,
      route_length(route, closed),
      order,
      route,
  };
}

SolveConfig read_config_from_stdin() {
  SolveConfig config;
  std::string line;

  auto read_required_line = [&line]() {
    if (!std::getline(std::cin, line)) {
      throw std::runtime_error("Unexpected end of input.");
    }
    line = trim(line);
    if (line.empty()) throw std::runtime_error("Encountered an empty input line.");
  };

  read_required_line();
  {
    std::istringstream stream(line);
    std::string key;
    std::string value;
    stream >> key >> value;
    if (key != "task" || value.empty()) throw std::runtime_error("Expected: task <taskKey>");
    config.task = parse_task(value);
  }

  read_required_line();
  {
    std::istringstream stream(line);
    std::string key;
    stream >> key >> config.algorithm_key;
    if (key != "algorithm" || config.algorithm_key.empty()) {
      throw std::runtime_error("Expected: algorithm <algorithmKey>");
    }
  }

  read_required_line();
  {
    std::istringstream stream(line);
    std::string key;
    stream >> key >> config.seed;
    if (key != "seed" || stream.fail()) throw std::runtime_error("Expected: seed <uint32>");
  }

  read_required_line();
  {
    std::istringstream stream(line);
    std::string key;
    stream >> key >> config.params.nests >> config.params.pa >> config.params.max_iter >>
        config.params.alpha >> config.params.beta;
    if (key != "params" || stream.fail()) {
      throw std::runtime_error("Expected: params <nests> <pa> <max_iter> <alpha> <beta>");
    }
  }

  int count = 0;
  read_required_line();
  {
    std::istringstream stream(line);
    std::string key;
    stream >> key >> count;
    if (key != "count" || stream.fail() || count < 0) {
      throw std::runtime_error("Expected: count <non-negative integer>");
    }
  }

  config.points.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    read_required_line();
    std::istringstream point_stream(line);
    InputPoint point;
    point_stream >> point.x >> point.y;
    if (point_stream.fail()) {
      throw std::runtime_error("Expected point line: <x> <y>");
    }
    config.points.push_back(point);
  }

  return config;
}

void write_success(const SolveOutput& output) {
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "status ok\n";
  std::cout << "closed " << (output.closed ? 1 : 0) << "\n";
  std::cout << "length " << output.length << "\n";
  std::cout << "order";
  for (int index : output.order) std::cout << " " << index;
  std::cout << "\n";
  std::cout << "route_count " << output.route.size() << "\n";
  for (const InputPoint& point : output.route) {
    std::cout << point.x << " " << point.y << "\n";
  }
}

void write_error(const std::string& message) {
  std::cout << "status error\n";
  std::cout << "message " << message << "\n";
}

}  // namespace

int main() {
  try {
    const SolveConfig config = read_config_from_stdin();
    const SolveOutput output = solve_route(config);
    write_success(output);
    return 0;
  } catch (const std::exception& error) {
    write_error(error.what());
    return 1;
  }
}
