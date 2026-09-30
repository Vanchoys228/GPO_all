#pragma once

#include "tsp/types.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace tsp {

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

double gamma_lanczos(double z);
std::vector<double> levy_vector(double beta, int size, SeededRng& rng);

}  // namespace tsp
