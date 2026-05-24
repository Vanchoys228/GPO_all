#include "tsp/algorithms.h"

#include <algorithm>
#include <cmath>

namespace tsp {

AlgorithmRun run_scatter_search(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed) {
  SeededRng rng(seed);
  const int ref_size = clamp(
      static_cast<int>(std::lround(params.pa * static_cast<double>(params.nests))),
      6,
      std::min(40, params.nests));
  const double mutation_rate = clamp(params.alpha * 0.5, 0.05, 0.45);
  const int local_steps = clamp(static_cast<int>(std::lround(params.beta)), 2, 30);
  const int max_no_improve = std::max(5, static_cast<int>(std::floor(params.max_iter * 0.25)));

  std::vector<Tour> population = build_initial_population(params.nests, problem, rng, 5);
  std::vector<Tour> ref_set = build_ref_set(population, ref_size);
  Tour best = clone_tour(pick_best(ref_set));
  std::vector<int> history = {best.length};
  int no_improve = 0;

  for (int iteration = 1; iteration <= params.max_iter && no_improve < max_no_improve; ++iteration) {
    struct PairInfo {
      int left = 0;
      int right = 0;
      int diversity = 0;
    };

    std::vector<PairInfo> pairs;
    for (int left = 0; left < static_cast<int>(ref_set.size()); ++left) {
      for (int right = left + 1; right < static_cast<int>(ref_set.size()); ++right) {
        pairs.push_back(PairInfo{
            left,
            right,
            order_hamming_distance(ref_set[left].order, ref_set[right].order)});
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const PairInfo& left, const PairInfo& right) {
      return left.diversity > right.diversity;
    });

    std::vector<Tour> children;
    const int limit = std::min(28, static_cast<int>(pairs.size()));
    for (int index = 0; index < limit; ++index) {
      const Tour& first = ref_set[pairs[index].left];
      const Tour& second = ref_set[pairs[index].right];
      std::vector<std::vector<int>> orders = {
          mutate_order(ordered_crossover(first.order, second.order, rng), rng, mutation_rate),
          mutate_order(ordered_crossover(second.order, first.order, rng), rng, mutation_rate),
          mutate_order(first.order, rng, mutation_rate * 0.75)};
      for (const std::vector<int>& order : orders) {
        children.push_back(two_opt_improve(order, problem, local_steps));
      }
    }

    population.insert(population.end(), children.begin(), children.end());
    std::sort(population.begin(), population.end(), [](const Tour& left, const Tour& right) {
      return left.length < right.length;
    });
    if (static_cast<int>(population.size()) > params.nests * 2) population.resize(params.nests * 2);

    ref_set = build_ref_set(population, ref_size);
    const Tour& iteration_best = pick_best(ref_set);
    if (iteration_best.length < best.length) {
      best = clone_tour(iteration_best);
      no_improve = 0;
    } else {
      ++no_improve;
    }
    history.push_back(best.length);
  }

  return AlgorithmRun{best, history, static_cast<int>(history.size()) - 1};
}

}  // namespace tsp
