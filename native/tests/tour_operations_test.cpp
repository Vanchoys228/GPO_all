#include "tsp/tour_operations.h"

#include <algorithm>
#include <cassert>
#include <vector>

int main() {
  tsp::Problem problem;
  problem.cities.resize(4);
  problem.distance_matrix = {
      {0, 1, 4, 1}, {1, 0, 1, 4}, {4, 1, 0, 1}, {1, 4, 1, 0}};
  assert(tsp::route_length({0, 1, 2, 3}, problem) == 4);

  std::vector<int> order{0, 1, 2, 3};
  tsp::reverse_segment(order, 1, 3);
  assert((order == std::vector<int>{0, 3, 2, 1}));
  tsp::swap_positions(order, 1, 3);
  assert((order == std::vector<int>{0, 1, 2, 3}));

  tsp::SeededRng rng(12U);
  const std::vector<int> child = tsp::ordered_crossover(
      {0, 1, 2, 3}, {0, 3, 1, 2}, rng);
  std::vector<int> sorted = child;
  std::sort(sorted.begin(), sorted.end());
  assert((sorted == std::vector<int>{0, 1, 2, 3}));
}
