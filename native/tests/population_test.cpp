#include "tsp/population.h"

#include <cassert>
#include <vector>

int main() {
  tsp::Problem problem;
  problem.cities.resize(4);
  problem.distance_matrix = {
      {0, 1, 4, 1}, {1, 0, 1, 4}, {4, 1, 0, 1}, {1, 4, 1, 0}};
  tsp::SeededRng rng(17U);
  const std::vector<tsp::Tour> population = tsp::build_initial_population(6, problem, rng, 2);
  assert(population.size() == 6U);
  assert(tsp::pick_best(population).length <= population.front().length);
  assert(!tsp::select_promising_seeds(population, 2).empty());
  assert(tsp::build_ref_set(population, 3).size() <= 3U);
}
