#include "tsp/algorithms.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace tsp {

AlgorithmRun run_ga_tabu(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed) {
  SeededRng rng(seed);
  const int restart_count = clamp(static_cast<int>(std::lround(std::sqrt(params.nests))), 2, 16);
  const int tabu_iterations = std::max(1, static_cast<int>(std::lround(params.beta)));
  const int tabu_tenure = clamp(static_cast<int>(std::lround(std::sqrt(params.nests) * 1.7)), 4, 25);
  const int stall_limit = std::max(10, static_cast<int>(std::floor(params.max_iter * 0.2)));

  std::vector<Tour> population = build_initial_population(params.nests, problem, rng, 6);
  Tour best = clone_tour(pick_best(population));
  std::vector<int> history = {best.length};
  int no_improve = 0;

  for (int generation = 0; generation < params.max_iter && no_improve < stall_limit; ++generation) {
    std::vector<Tour> sorted = population;
    std::sort(sorted.begin(), sorted.end(), [](const Tour& left, const Tour& right) {
      return left.length < right.length;
    });

    std::vector<Tour> next_population;
    next_population.reserve(params.nests);
    const int elite_count = std::max(3, static_cast<int>(std::floor(params.nests * 0.1)));
    for (int index = 0; index < elite_count && index < static_cast<int>(sorted.size()); ++index) {
      next_population.push_back(clone_tour(sorted[index]));
    }

    while (static_cast<int>(next_population.size()) < params.nests) {
      const Tour& parent_a = select_tournament(sorted, rng);
      const Tour& parent_b = select_tournament(sorted, rng);
      std::vector<int> child =
          rng.chance(params.alpha) ? ordered_crossover(parent_a.order, parent_b.order, rng)
                                   : parent_a.order;
      child = mutate_order(child, rng, params.pa);
      next_population.push_back(two_opt_improve(child, problem, rng.chance(0.35) ? 6 : 2));
    }

    population = std::move(next_population);
    const Tour& generation_best = pick_best(population);
    if (generation_best.length < best.length) {
      best = clone_tour(generation_best);
      no_improve = 0;
    } else {
      ++no_improve;
    }
    history.push_back(best.length);
  }

  const std::vector<Tour> seeds = select_promising_seeds(population, restart_count);
  for (const Tour& start : seeds) {
    Tour current = clone_tour(start);
    std::unordered_map<std::string, int> tabu;

    for (int iteration = 0; iteration < tabu_iterations; ++iteration) {
      Tour best_move;
      std::string best_key;
      bool has_best_move = false;

      for (int candidate_index = 0; candidate_index < 24; ++candidate_index) {
        const int left = rng.uniform_int(1, static_cast<int>(current.order.size()) - 2);
        const int right = rng.uniform_int(left + 1, static_cast<int>(current.order.size()) - 1);
        const bool use_swap = rng.chance(0.35);

        std::vector<int> candidate_order = current.order;
        if (use_swap) swap_positions(candidate_order, left, right);
        else reverse_segment(candidate_order, left, right);

        Tour candidate = two_opt_improve(candidate_order, problem, use_swap ? 2 : 3);
        const std::string key = (use_swap ? "swap:" : "reverse:") + std::to_string(left) + ":" +
                                std::to_string(right);
        const bool is_tabu = tabu.contains(key) && tabu[key] > iteration;
        if (is_tabu && candidate.length >= best.length) continue;

        if (!has_best_move || candidate.length < best_move.length) {
          best_move = std::move(candidate);
          best_key = key;
          has_best_move = true;
        }
      }

      if (!has_best_move) break;

      current = best_move;
      tabu[best_key] = iteration + tabu_tenure;
      if (current.length < best.length) best = clone_tour(current);
      history.push_back(best.length);
    }
  }

  return AlgorithmRun{best, history, static_cast<int>(history.size()) - 1};
}

}  // namespace tsp
