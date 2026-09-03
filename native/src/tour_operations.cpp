#include "tsp/tour_operations.h"

#include <algorithm>
#include <unordered_set>

namespace tsp {
namespace {

std::vector<int> insert_position(const std::vector<int>& order, int from, int to) {
  std::vector<int> next = order;
  const int value = next[from];
  next.erase(next.begin() + from);
  next.insert(next.begin() + to, value);
  return next;
}

}  // namespace

int route_length(const std::vector<int>& order, const Problem& problem) {
  int total = 0;
  for (std::size_t index = 0; index < order.size(); ++index) {
    total += problem.distance_matrix[order[index]][order[(index + 1) % order.size()]];
  }
  return total;
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
    child[index] = genes_b[source++];
    used.insert(child[index]);
  }

  std::vector<int> order{0};
  order.reserve(parent_a.size());
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
    reverse_segment(candidate, left, rng.uniform_int(left + 1, last));
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

}  // namespace tsp
