#include "tsp/algorithms.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace tsp {

namespace {

double average(const std::vector<double>& values) {
  double sum = 0.0;
  for (double value : values) sum += value;
  return values.empty() ? 0.0 : sum / static_cast<double>(values.size());
}

double estimate_initial_temperature(const Tour& solution, const Problem& problem, SeededRng& rng) {
  std::vector<double> deltas;
  deltas.reserve(80);
  for (int index = 0; index < 80; ++index) {
    const int candidate_length = route_length(create_neighbor(solution.order, rng), problem);
    const int delta = candidate_length - solution.length;
    if (delta > 0) deltas.push_back(static_cast<double>(delta));
  }

  const double mean_positive = deltas.empty() ? 50.0 : average(deltas);
  return mean_positive / std::abs(std::log(0.8));
}

}  // namespace

AlgorithmRun run_annealing(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed) {
  SeededRng rng(seed);
  const std::vector<Tour> pool = build_initial_population(params.nests, problem, rng, 6);
  std::vector<Tour> sorted_pool = pool;
  std::sort(sorted_pool.begin(), sorted_pool.end(), [](const Tour& left, const Tour& right) {
    return left.length < right.length;
  });

  std::vector<Tour> elite_starts;
  for (int index = 0; index < std::min(8, static_cast<int>(sorted_pool.size())); ++index) {
    elite_starts.push_back(clone_tour(sorted_pool[index]));
  }

  Tour current = clone_tour(pick_best(pool));
  Tour best = clone_tour(current);
  const double initial_temperature = estimate_initial_temperature(current, problem, rng) * 2.5;
  double temperature = initial_temperature;
  const double minimum_temperature = std::max(1e-6, params.pa);
  double alpha = params.alpha;
  int accepted_window = 0;
  int stagnation = 0;
  std::vector<int> history = {best.length};
  const int move_strength = clamp(static_cast<int>(std::lround(3.0 + params.beta * 3.0)), 3, 12);
  const int local_moves = clamp(static_cast<int>(std::lround(6.0 + params.beta * 5.0)), 6, 20);

  for (int iteration = 1; iteration <= params.max_iter && temperature > minimum_temperature; ++iteration) {
    const std::vector<int>& candidate_base =
        stagnation > 50 && rng.chance(0.5) ? best.order : current.order;

    const CandidateBuildOptions options{
        stagnation > 80 ? 6 : 3,
        move_strength,
        local_moves,
        stagnation > 80 ? 0.7 : 0.22};
    Tour candidate = build_candidate_from_order(candidate_base, problem, rng, options);

    const int delta = candidate.length - current.length;
    const double scaled_temperature =
        temperature * (1.0 + static_cast<double>(problem.cities.size()) * 0.02);
    if (delta < 0 || rng.uniform() < std::exp(-static_cast<double>(delta) / std::max(scaled_temperature, 1e-9))) {
      current = candidate;
      ++accepted_window;
    }

    if (current.length < best.length) {
      best = clone_tour(current);
      stagnation = 0;
    } else {
      ++stagnation;
    }

    history.push_back(best.length);

    if (stagnation > 140) {
      const std::vector<int>& restart_base =
          rng.chance(0.65)
              ? best.order
              : elite_starts[rng.uniform_int(0, static_cast<int>(elite_starts.size()) - 1)].order;

      current = build_candidate_from_order(
          restart_base,
          problem,
          rng,
          CandidateBuildOptions{3, move_strength + 3, local_moves + 4, 0.9});
      temperature = std::max(temperature, initial_temperature * 0.45);
      stagnation = 0;
    }

    if (iteration % 100 == 0) {
      const double ratio = static_cast<double>(accepted_window) / 100.0;
      if (ratio > 0.6) alpha = clamp(alpha * 0.992, 0.9, 0.99995);
      else if (ratio < 0.12) alpha = clamp(alpha * 1.004, 0.9, 0.99995);
      accepted_window = 0;
    }

    temperature *= alpha;
  }

  return AlgorithmRun{best, history, static_cast<int>(history.size()) - 1};
}

}  // namespace tsp
