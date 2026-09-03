#pragma once

#include "gpo/route_solver_types.h"

#include <vector>

namespace gpo {

struct AugmentedProblem {
  tsp::Problem problem;
  int dummy_index = -1;
};

tsp::Problem build_euclidean_problem(const std::vector<InputPoint>& points);
AugmentedProblem build_augmented_problem(
    const std::vector<InputPoint>& points,
    TaskKind task);
std::vector<int> extract_open_order_from_augmented_cycle(
    const std::vector<int>& cycle_order,
    int dummy_index);
std::vector<int> orient_fixed_endpoints(
    std::vector<int> order,
    int start_node,
    int end_node);
double calculate_route_length(const std::vector<InputPoint>& route, bool closed);

}  // namespace gpo
