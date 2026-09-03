#include "tsp/local_search.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tsp {

std::vector<int> nearest_neighbor_order(const Problem& problem) {
  const int size = static_cast<int>(problem.cities.size());
  std::vector<bool> used(size, false);
  std::vector<int> order;
  order.reserve(size);
  order.push_back(0);
  used[0] = true;
  int current = 0;
  while (static_cast<int>(order.size()) < size) {
    int best_node = -1;
    int best_distance = std::numeric_limits<int>::max();
    for (int candidate = 1; candidate < size; ++candidate) {
      if (used[candidate]) continue;
      const int distance = problem.distance_matrix[current][candidate];
      if (distance < best_distance) {
        best_distance = distance;
        best_node = candidate;
      }
    }
    used[best_node] = true;
    order.push_back(best_node);
    current = best_node;
  }
  return order;
}

Tour two_opt_improve(const std::vector<int>& order, const Problem& problem, int max_moves) {
  std::vector<int> candidate = order;
  int current_length = route_length(candidate, problem);
  int moves = 0;
  while (moves < max_moves) {
    bool improved = false;
    for (int left = 1; left < static_cast<int>(candidate.size()) - 1; ++left) {
      const int previous = candidate[left - 1];
      const int current = candidate[left];
      for (int right = left + 1; right < static_cast<int>(candidate.size()); ++right) {
        const int tail = candidate[right];
        const int next = candidate[(right + 1) % candidate.size()];
        const int delta = problem.distance_matrix[previous][tail] +
            problem.distance_matrix[current][next] -
            problem.distance_matrix[previous][current] -
            problem.distance_matrix[tail][next];
        if (delta < 0) {
          reverse_segment(candidate, left, right);
          current_length += delta;
          ++moves;
          improved = true;
          break;
        }
      }
      if (improved) break;
    }
    if (!improved) break;
  }
  return Tour{candidate, current_length};
}

std::vector<int> double_bridge_kick(const std::vector<int>& order, SeededRng& rng) {
  if (order.size() < 9) return create_neighbor(order, rng);
  const int last = static_cast<int>(order.size()) - 1;
  std::vector<int> cuts = {
      rng.uniform_int(1, std::max(2, static_cast<int>(std::floor(last * 0.2)))),
      rng.uniform_int(std::max(2, static_cast<int>(std::floor(last * 0.2))),
                      std::max(3, static_cast<int>(std::floor(last * 0.45)))),
      rng.uniform_int(std::max(3, static_cast<int>(std::floor(last * 0.45))),
                      std::max(4, static_cast<int>(std::floor(last * 0.7)))),
      rng.uniform_int(std::max(4, static_cast<int>(std::floor(last * 0.7))), last)};
  std::sort(cuts.begin(), cuts.end());
  const int a = cuts[0];
  const int b = cuts[1];
  const int c = cuts[2];
  const int d = cuts[3];

  std::vector<int> kicked{0};
  kicked.reserve(order.size());
  kicked.insert(kicked.end(), order.begin() + 1, order.begin() + a);
  kicked.insert(kicked.end(), order.begin() + c, order.begin() + d);
  kicked.insert(kicked.end(), order.begin() + b, order.begin() + c);
  kicked.insert(kicked.end(), order.begin() + a, order.begin() + b);
  kicked.insert(kicked.end(), order.begin() + d, order.end());
  return kicked;
}

Tour build_candidate_from_order(
    const std::vector<int>& base_order,
    const Problem& problem,
    SeededRng& rng,
    const CandidateBuildOptions& options) {
  Tour best;
  bool has_best = false;
  for (int trial = 0; trial < options.trials; ++trial) {
    std::vector<int> candidate = base_order;
    if (rng.chance(options.kick_probability)) candidate = double_bridge_kick(candidate, rng);
    const int steps = rng.uniform_int(1, std::max(1, options.move_strength));
    for (int step = 0; step < steps; ++step) candidate = create_neighbor(candidate, rng);
    Tour improved = two_opt_improve(candidate, problem, std::max(1, options.local_moves));
    if (!has_best || improved.length < best.length) {
      best = clone_tour(improved);
      has_best = true;
    }
  }
  return best;
}

}  // namespace tsp
