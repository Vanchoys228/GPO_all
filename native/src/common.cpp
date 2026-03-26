#include "tsp/common.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <unordered_set>

namespace tsp {

namespace {

std::uint32_t imul(std::uint32_t a, std::uint32_t b) {
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(a) * static_cast<std::uint64_t>(b));
}

}  // namespace

SeededRng::SeededRng(std::uint32_t seed) : state_(seed == 0 ? 0x6d2b79f5U : seed) {}

double SeededRng::uniform() {
  state_ += 0x6d2b79f5U;
  std::uint32_t value = state_;
  value = imul(value ^ (value >> 15U), value | 1U);
  value ^= value + imul(value ^ (value >> 7U), value | 61U);
  value = value ^ (value >> 14U);
  return static_cast<double>(value) / 4294967296.0;
}

int SeededRng::uniform_int(int min_value, int max_value) {
  if (max_value <= min_value) return min_value;
  const double span = static_cast<double>(max_value - min_value + 1);
  return min_value + static_cast<int>(std::floor(uniform() * span));
}

bool SeededRng::chance(double probability) {
  return uniform() < clamp(probability, 0.0, 1.0);
}

double SeededRng::gaussian() {
  if (has_cached_gaussian_) {
    has_cached_gaussian_ = false;
    return cached_gaussian_;
  }

  double u = 0.0;
  double v = 0.0;
  while (u == 0.0) u = uniform();
  while (v == 0.0) v = uniform();

  const double radius = std::sqrt(-2.0 * std::log(u));
  const double theta = 2.0 * 3.14159265358979323846 * v;
  cached_gaussian_ = radius * std::sin(theta);
  has_cached_gaussian_ = true;
  return radius * std::cos(theta);
}

int round_distance(double value) {
  return static_cast<int>(std::floor(value + 0.5));
}

double gamma_lanczos(double z) {
  static const double coefficients[] = {
      676.5203681218851,
      -1259.1392167224028,
      771.3234287776531,
      -176.6150291621406,
      12.507343278686905,
      -0.13857109526572012,
      9.984369578019572e-6,
      1.5056327351493116e-7};

  if (z < 0.5) {
    return 3.14159265358979323846 /
           (std::sin(3.14159265358979323846 * z) * gamma_lanczos(1.0 - z));
  }

  double x = 0.9999999999998099;
  const double shifted = z - 1.0;
  for (int index = 0; index < 8; ++index) x += coefficients[index] / (shifted + index + 1.0);
  const double t = shifted + 7.5;
  return std::sqrt(2.0 * 3.14159265358979323846) * std::pow(t, shifted + 0.5) *
         std::exp(-t) * x;
}

std::vector<double> levy_vector(double beta, int size, SeededRng& rng) {
  const double safe_beta = clamp(beta, 1.1, 1.99);
  const double numerator =
      gamma_lanczos(1.0 + safe_beta) * std::sin((3.14159265358979323846 * safe_beta) / 2.0);
  const double denominator =
      gamma_lanczos((1.0 + safe_beta) / 2.0) * safe_beta *
      std::pow(2.0, (safe_beta - 1.0) / 2.0);
  const double sigma = std::pow(numerator / denominator, 1.0 / safe_beta);

  std::vector<double> values(size, 0.0);
  for (int index = 0; index < size; ++index) {
    const double u = rng.gaussian() * sigma;
    const double v = rng.gaussian();
    values[index] = u / std::pow(std::abs(v) + 1e-12, 1.0 / safe_beta);
  }
  return values;
}

int route_length(const std::vector<int>& order, const Problem& problem) {
  int total = 0;
  for (std::size_t index = 0; index < order.size(); ++index) {
    total += problem.distance_matrix[order[index]][order[(index + 1) % order.size()]];
  }
  return total;
}

Tour make_tour(std::vector<int> order, const Problem& problem) {
  Tour tour;
  tour.order = std::move(order);
  tour.length = route_length(tour.order, problem);
  return tour;
}

Tour clone_tour(const Tour& tour) {
  return Tour{tour.order, tour.length};
}

void reverse_segment(std::vector<int>& order, int start, int end) {
  while (start < end) {
    std::swap(order[start], order[end]);
    ++start;
    --end;
  }
}

void swap_positions(std::vector<int>& order, int left, int right) {
  std::swap(order[left], order[right]);
}

std::vector<int> insert_position(const std::vector<int>& order, int from, int to) {
  std::vector<int> next = order;
  const int value = next[from];
  next.erase(next.begin() + from);
  next.insert(next.begin() + to, value);
  return next;
}

std::vector<int> random_order(int size, SeededRng& rng) {
  std::vector<int> nodes;
  nodes.reserve(size);
  nodes.push_back(0);
  for (int city = 1; city < size; ++city) nodes.push_back(city);
  std::vector<int> tail(nodes.begin() + 1, nodes.end());
  rng.shuffle(tail);
  std::copy(tail.begin(), tail.end(), nodes.begin() + 1);
  return nodes;
}

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

int order_hamming_distance(const std::vector<int>& first, const std::vector<int>& second) {
  int distance = 0;
  for (std::size_t index = 1; index < first.size(); ++index) {
    if (first[index] != second[index]) ++distance;
  }
  return distance;
}

std::vector<int> ordered_crossover(
    const std::vector<int>& parent_a,
    const std::vector<int>& parent_b,
    SeededRng& rng) {
  std::vector<int> genes_a(parent_a.begin() + 1, parent_a.end());
  std::vector<int> genes_b(parent_b.begin() + 1, parent_b.end());
  std::vector<int> child(genes_a.size(), -1);
  const int left = rng.uniform_int(0, static_cast<int>(genes_a.size()) - 1);
  const int right = rng.uniform_int(left, static_cast<int>(genes_a.size()) - 1);
  std::unordered_set<int> used;

  for (int index = left; index <= right; ++index) {
    child[index] = genes_a[index];
    used.insert(genes_a[index]);
  }

  int source = 0;
  for (std::size_t index = 0; index < child.size(); ++index) {
    if (child[index] != -1) continue;
    while (used.count(genes_b[source]) > 0) ++source;
    child[index] = genes_b[source];
    used.insert(genes_b[source]);
    ++source;
  }

  std::vector<int> order;
  order.reserve(parent_a.size());
  order.push_back(0);
  order.insert(order.end(), child.begin(), child.end());
  return order;
}

std::vector<int> mutate_order(
    const std::vector<int>& order,
    SeededRng& rng,
    double mutation_rate) {
  std::vector<int> candidate = order;
  const int last = static_cast<int>(candidate.size()) - 1;

  if (rng.chance(mutation_rate)) {
    const int left = rng.uniform_int(1, last - 1);
    const int right = rng.uniform_int(left + 1, last);
    reverse_segment(candidate, left, right);
  }
  if (rng.chance(mutation_rate)) {
    const int left = rng.uniform_int(1, last);
    int right = rng.uniform_int(1, last);
    if (left == right) right = right == last ? 1 : right + 1;
    swap_positions(candidate, left, right);
  }
  if (rng.chance(mutation_rate * 0.4)) {
    const int from = rng.uniform_int(1, last);
    int to = rng.uniform_int(1, last);
    if (from == to) to = to == last ? 1 : to + 1;
    return insert_position(candidate, from, to);
  }

  return candidate;
}

std::vector<int> create_neighbor(const std::vector<int>& order, SeededRng& rng) {
  std::vector<int> candidate = order;
  const int last = static_cast<int>(candidate.size()) - 1;
  const double move = rng.uniform();
  if (move < 0.4) {
    const int left = rng.uniform_int(1, last - 1);
    const int right = rng.uniform_int(left + 1, last);
    reverse_segment(candidate, left, right);
    return candidate;
  }
  if (move < 0.75) {
    const int left = rng.uniform_int(1, last);
    int right = rng.uniform_int(1, last);
    if (left == right) right = right == last ? 1 : right + 1;
    swap_positions(candidate, left, right);
    return candidate;
  }

  const int from = rng.uniform_int(1, last);
  int to = rng.uniform_int(1, last);
  if (from == to) to = to == last ? 1 : to + 1;
  return insert_position(candidate, from, to);
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
        const int delta =
            problem.distance_matrix[previous][tail] +
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

std::vector<Tour> build_initial_population(
    int size,
    const Problem& problem,
    SeededRng& rng,
    int local_moves) {
  std::vector<Tour> population;
  population.reserve(size);
  population.push_back(two_opt_improve(nearest_neighbor_order(problem), problem, local_moves + 4));
  while (static_cast<int>(population.size()) < size) {
    population.push_back(two_opt_improve(random_order(static_cast<int>(problem.cities.size()), rng), problem, local_moves));
  }
  return population;
}

const Tour& pick_best(const std::vector<Tour>& population) {
  return *std::min_element(
      population.begin(),
      population.end(),
      [](const Tour& left, const Tour& right) { return left.length < right.length; });
}

const Tour& select_tournament(const std::vector<Tour>& population, SeededRng& rng, int size) {
  int best_index = rng.uniform_int(0, static_cast<int>(population.size()) - 1);
  for (int index = 1; index < size; ++index) {
    const int candidate_index = rng.uniform_int(0, static_cast<int>(population.size()) - 1);
    if (population[candidate_index].length < population[best_index].length) best_index = candidate_index;
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

  std::vector<Tour> pool(unique.begin() + std::min(elite_count, static_cast<int>(unique.size())), unique.end());
  while (static_cast<int>(selected.size()) < ref_size && !pool.empty()) {
    int best_index = 0;
    int best_score = -1;
    for (int index = 0; index < static_cast<int>(pool.size()); ++index) {
      int min_distance = std::numeric_limits<int>::max();
      for (const Tour& current : selected) {
        min_distance = std::min(min_distance, order_hamming_distance(current.order, pool[index].order));
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

std::vector<double> order_to_keys(const std::vector<int>& order) {
  std::vector<double> keys(order.size() - 1, 0.0);
  for (std::size_t index = 1; index < order.size(); ++index) {
    keys[order[index] - 1] = static_cast<double>(index) / static_cast<double>(order.size());
  }
  return keys;
}

std::vector<int> keys_to_order(const std::vector<double>& keys) {
  struct KeyItem {
    int city = 0;
    double value = 0.0;
  };

  std::vector<KeyItem> items;
  items.reserve(keys.size());
  for (std::size_t index = 0; index < keys.size(); ++index) {
    items.push_back(KeyItem{static_cast<int>(index) + 1, keys[index]});
  }
  std::sort(items.begin(), items.end(), [](const KeyItem& left, const KeyItem& right) {
    return left.value < right.value;
  });

  std::vector<int> order;
  order.reserve(keys.size() + 1);
  order.push_back(0);
  for (const KeyItem& item : items) order.push_back(item.city);
  return order;
}

Tour evaluate_keys(const std::vector<double>& keys, const Problem& problem, int local_moves) {
  return two_opt_improve(keys_to_order(keys), problem, local_moves);
}

std::vector<int> double_bridge_kick(const std::vector<int>& order, SeededRng& rng) {
  if (order.size() < 9) return create_neighbor(order, rng);

  const int last = static_cast<int>(order.size()) - 1;
  std::vector<int> cuts = {
      rng.uniform_int(1, std::max(2, static_cast<int>(std::floor(last * 0.2)))),
      rng.uniform_int(
          std::max(2, static_cast<int>(std::floor(last * 0.2))),
          std::max(3, static_cast<int>(std::floor(last * 0.45)))),
      rng.uniform_int(
          std::max(3, static_cast<int>(std::floor(last * 0.45))),
          std::max(4, static_cast<int>(std::floor(last * 0.7)))),
      rng.uniform_int(
          std::max(4, static_cast<int>(std::floor(last * 0.7))),
          last)};
  std::sort(cuts.begin(), cuts.end());

  const int a = cuts[0];
  const int b = cuts[1];
  const int c = cuts[2];
  const int d = cuts[3];

  std::vector<int> p1(order.begin() + 1, order.begin() + a);
  std::vector<int> p2(order.begin() + a, order.begin() + b);
  std::vector<int> p3(order.begin() + b, order.begin() + c);
  std::vector<int> p4(order.begin() + c, order.begin() + d);
  std::vector<int> p5(order.begin() + d, order.end());

  std::vector<int> kicked;
  kicked.reserve(order.size());
  kicked.push_back(0);
  kicked.insert(kicked.end(), p1.begin(), p1.end());
  kicked.insert(kicked.end(), p4.begin(), p4.end());
  kicked.insert(kicked.end(), p3.begin(), p3.end());
  kicked.insert(kicked.end(), p2.begin(), p2.end());
  kicked.insert(kicked.end(), p5.begin(), p5.end());
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
