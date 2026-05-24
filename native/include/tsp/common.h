#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace tsp {

struct City {
  int id = 0;
  double x = 0.0;
  double y = 0.0;
};

struct Problem {
  std::string name;
  std::vector<City> cities;
  std::vector<std::vector<int>> distance_matrix;
};

struct Tour {
  std::vector<int> order;
  int length = 0;
};

struct AlgorithmParams {
  int nests = 0;
  double pa = 0.0;
  int max_iter = 0;
  double alpha = 0.0;
  double beta = 0.0;
};

struct AlgorithmRun {
  Tour best;
  std::vector<int> history;
  int iterations = 0;
};

template <typename T>
T clamp(T value, T min_value, T max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

class SeededRng {
 public:
  explicit SeededRng(std::uint32_t seed);

  double uniform();
  int uniform_int(int min_value, int max_value);
  bool chance(double probability);
  double gaussian();

  template <typename T>
  void shuffle(std::vector<T>& values) {
    for (int index = static_cast<int>(values.size()) - 1; index > 0; --index) {
      const int swap_index = uniform_int(0, index);
      if (swap_index == index) continue;
      T temp = std::move(values[index]);
      values[index] = std::move(values[swap_index]);
      values[swap_index] = std::move(temp);
    }
  }

 private:
  std::uint32_t state_;
  bool has_cached_gaussian_ = false;
  double cached_gaussian_ = 0.0;
};

int round_distance(double value);
double gamma_lanczos(double z);
std::vector<double> levy_vector(double beta, int size, SeededRng& rng);

int route_length(const std::vector<int>& order, const Problem& problem);
Tour make_tour(std::vector<int> order, const Problem& problem);
Tour clone_tour(const Tour& tour);

void reverse_segment(std::vector<int>& order, int start, int end);
void swap_positions(std::vector<int>& order, int left, int right);
std::vector<int> insert_position(const std::vector<int>& order, int from, int to);

std::vector<int> random_order(int size, SeededRng& rng);
std::vector<int> nearest_neighbor_order(const Problem& problem);
int order_hamming_distance(const std::vector<int>& first, const std::vector<int>& second);
std::vector<int> ordered_crossover(
    const std::vector<int>& parent_a,
    const std::vector<int>& parent_b,
    SeededRng& rng);
std::vector<int> mutate_order(
    const std::vector<int>& order,
    SeededRng& rng,
    double mutation_rate);
std::vector<int> create_neighbor(const std::vector<int>& order, SeededRng& rng);
Tour two_opt_improve(const std::vector<int>& order, const Problem& problem, int max_moves = 6);

std::vector<Tour> build_initial_population(
    int size,
    const Problem& problem,
    SeededRng& rng,
    int local_moves);
const Tour& pick_best(const std::vector<Tour>& population);
const Tour& select_tournament(const std::vector<Tour>& population, SeededRng& rng, int size = 4);
std::vector<Tour> select_promising_seeds(const std::vector<Tour>& population, int count);
std::vector<Tour> build_ref_set(const std::vector<Tour>& population, int ref_size);

std::vector<double> order_to_keys(const std::vector<int>& order);
std::vector<int> keys_to_order(const std::vector<double>& keys);
Tour evaluate_keys(const std::vector<double>& keys, const Problem& problem, int local_moves);

struct CandidateBuildOptions {
  int trials = 1;
  int move_strength = 3;
  int local_moves = 6;
  double kick_probability = 0.15;
};

std::vector<int> double_bridge_kick(const std::vector<int>& order, SeededRng& rng);
Tour build_candidate_from_order(
    const std::vector<int>& base_order,
    const Problem& problem,
    SeededRng& rng,
    const CandidateBuildOptions& options);

}  // namespace tsp
