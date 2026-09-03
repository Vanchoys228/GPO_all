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

}  // namespace tsp
