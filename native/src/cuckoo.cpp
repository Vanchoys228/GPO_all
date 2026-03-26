#include "tsp/algorithms.h"

#include <algorithm>
#include <cmath>

namespace tsp {

AlgorithmRun run_cuckoo_search(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed) {
  SeededRng rng(seed);
  const double levy_beta = clamp(params.beta, 1.1, 1.99);
  const int abandon_count = std::max(1, static_cast<int>(std::floor(params.pa * params.nests)));
  const int base_local_moves = clamp(static_cast<int>(std::lround(5.0 + params.alpha * 24.0)), 5, 16);

  std::vector<Tour> nests = build_initial_population(params.nests, problem, rng, 5);
  Tour best = clone_tour(pick_best(nests));
  std::vector<int> history = {best.length};

  for (int iteration = 1; iteration <= params.max_iter; ++iteration) {
    for (int index = 0; index < static_cast<int>(nests.size()); ++index) {
      const Tour& source = nests[index];
      std::vector<int> candidate_order =
          rng.chance(0.45) ? ordered_crossover(source.order, best.order, rng) : source.order;

      const double levy_step = std::abs(levy_vector(levy_beta, 1, rng)[0]);
      const int mutation_count = clamp(static_cast<int>(std::lround(1.0 + levy_step * 4.0)), 1, 18);
      if (rng.chance(0.3)) candidate_order = double_bridge_kick(candidate_order, rng);
      for (int step = 0; step < mutation_count; ++step) {
        candidate_order = create_neighbor(candidate_order, rng);
      }

      Tour candidate = two_opt_improve(
          candidate_order,
          problem,
          clamp(base_local_moves + mutation_count / 3, base_local_moves, 24));
      const int target_index = rng.uniform_int(0, static_cast<int>(nests.size()) - 1);
      if (candidate.length < nests[target_index].length) nests[target_index] = std::move(candidate);
    }

    struct RankedNest {
      int index = 0;
      int length = 0;
    };
    std::vector<RankedNest> worst;
    worst.reserve(nests.size());
    for (int index = 0; index < static_cast<int>(nests.size()); ++index) {
      worst.push_back(RankedNest{index, nests[index].length});
    }
    std::sort(worst.begin(), worst.end(), [](const RankedNest& left, const RankedNest& right) {
      return left.length > right.length;
    });
    if (static_cast<int>(worst.size()) > abandon_count) worst.resize(abandon_count);

    for (const RankedNest& item : worst) {
      const std::vector<int>& replacement_base =
          rng.chance(0.7) ? best.order : random_order(static_cast<int>(problem.cities.size()), rng);
      nests[item.index] = build_candidate_from_order(
          replacement_base,
          problem,
          rng,
          CandidateBuildOptions{
              3,
              clamp(static_cast<int>(std::lround(3.0 + params.alpha * 30.0)), 3, 10),
              base_local_moves + 2,
              0.65});
    }

    const Tour& iteration_best = pick_best(nests);
    if (iteration_best.length < best.length) best = clone_tour(iteration_best);
    history.push_back(best.length);
  }

  return AlgorithmRun{best, history, static_cast<int>(history.size()) - 1};
}

}  // namespace tsp
