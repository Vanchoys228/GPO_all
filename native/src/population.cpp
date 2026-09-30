#include "tsp/population.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>

namespace tsp {

std::vector<Tour> build_initial_population(
    int size,
    const Problem& problem,
    SeededRng& rng,
    int local_moves) {
  std::vector<Tour> population;
  population.reserve(size);
  population.push_back(
      two_opt_improve(nearest_neighbor_order(problem), problem, local_moves + 4));
  while (static_cast<int>(population.size()) < size) {
    population.push_back(two_opt_improve(
        random_order(static_cast<int>(problem.cities.size()), rng), problem, local_moves));
  }
  return population;
}

const Tour& pick_best(const std::vector<Tour>& population) {
  return *std::min_element(
      population.begin(),
      population.end(),
      [](const Tour& left, const Tour& right) { return left.length < right.length; });
}

const Tour& select_tournament(
    const std::vector<Tour>& population,
    SeededRng& rng,
    int size) {
  int best_index = rng.uniform_int(0, static_cast<int>(population.size()) - 1);
  for (int index = 1; index < size; ++index) {
    const int candidate_index = rng.uniform_int(0, static_cast<int>(population.size()) - 1);
    if (population[candidate_index].length < population[best_index].length) {
      best_index = candidate_index;
    }
  }
  return population[best_index];
}

std::vector<Tour> select_promising_seeds(const std::vector<Tour>& population, int count) {
  std::vector<Tour> sorted = population;
  std::sort(sorted.begin(), sorted.end(), [](const Tour& left, const Tour& right) {
    return left.length < right.length;
  });

  std::vector<Tour> selected;
  const int min_distance = std::max(6, static_cast<int>(sorted.front().order.size()) / 5);
  for (const Tour& candidate : sorted) {
    bool far_enough = true;
    for (const Tour& seed : selected) {
      if (order_hamming_distance(seed.order, candidate.order) < min_distance) {
        far_enough = false;
        break;
      }
    }
    if (far_enough) selected.push_back(clone_tour(candidate));
    if (static_cast<int>(selected.size()) >= count) break;
  }
  if (selected.empty()) {
    for (int index = 0; index < count && index < static_cast<int>(sorted.size()); ++index) {
      selected.push_back(clone_tour(sorted[index]));
    }
  }
  return selected;
}

std::vector<Tour> build_ref_set(const std::vector<Tour>& population, int ref_size) {
  std::vector<Tour> sorted = population;
  std::sort(sorted.begin(), sorted.end(), [](const Tour& left, const Tour& right) {
    return left.length < right.length;
  });

  std::unordered_set<std::string> seen;
  std::vector<Tour> unique;
  unique.reserve(sorted.size());
  for (const Tour& candidate : sorted) {
    std::string key;
    key.reserve(candidate.order.size() * 3);
    for (int node : candidate.order) {
      key += std::to_string(node);
      key.push_back('-');
    }
    if (seen.insert(key).second) unique.push_back(clone_tour(candidate));
  }

  const int elite_count = std::max(2, static_cast<int>(std::ceil(ref_size / 2.0)));
  std::vector<Tour> selected;
  for (int index = 0; index < elite_count && index < static_cast<int>(unique.size()); ++index) {
    selected.push_back(clone_tour(unique[index]));
  }

  std::vector<Tour> pool(
      unique.begin() + std::min(elite_count, static_cast<int>(unique.size())), unique.end());
  while (static_cast<int>(selected.size()) < ref_size && !pool.empty()) {
    int best_index = 0;
    int best_score = -1;
    for (int index = 0; index < static_cast<int>(pool.size()); ++index) {
      int min_distance = std::numeric_limits<int>::max();
      for (const Tour& current : selected) {
        min_distance = std::min(
            min_distance, order_hamming_distance(current.order, pool[index].order));
      }
      if (min_distance > best_score) {
        best_score = min_distance;
        best_index = index;
      }
    }
    selected.push_back(clone_tour(pool[best_index]));
    pool.erase(pool.begin() + best_index);
  }
  if (static_cast<int>(selected.size()) > ref_size) selected.resize(ref_size);
  return selected;
}

}  // namespace tsp
