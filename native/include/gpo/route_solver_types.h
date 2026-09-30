#pragma once

#include "tsp/common.h"

#include <cstdint>
#include <string>
#include <vector>

namespace gpo {

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

}  // namespace gpo
