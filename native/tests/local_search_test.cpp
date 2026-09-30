#include "tsp/local_search.h"

#include <algorithm>
#include <cassert>
#include <vector>

int main() {
  tsp::Problem problem;
  problem.cities.resize(4);
  problem.distance_matrix = {
      {0, 1, 4, 1}, {1, 0, 1, 4}, {4, 1, 0, 1}, {1, 4, 1, 0}};
  assert((tsp::nearest_neighbor_order(problem) == std::vector<int>{0, 1, 2, 3}));

  const std::vector<int> crossed{0, 1, 3, 2};
  const tsp::Tour improved = tsp::two_opt_improve(crossed, problem, 6);
  assert(improved.length <= tsp::route_length(crossed, problem));

  tsp::SeededRng rng(99U);
  const std::vector<int> kicked = tsp::double_bridge_kick(
      {0, 1, 2, 3, 4, 5, 6, 7, 8}, rng);
  std::vector<int> sorted = kicked;
  std::sort(sorted.begin(), sorted.end());
  assert((sorted == std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8}));
}
