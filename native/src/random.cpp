#include "tsp/random.h"

#include <cmath>

namespace tsp {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::uint32_t imul(std::uint32_t left, std::uint32_t right) {
  return static_cast<std::uint32_t>(
      static_cast<std::uint64_t>(left) * static_cast<std::uint64_t>(right));
}

}  // namespace

SeededRng::SeededRng(std::uint32_t seed) : state_(seed == 0 ? 0x6d2b79f5U : seed) {}

double SeededRng::uniform() {
  state_ += 0x6d2b79f5U;
  std::uint32_t value = state_;
  value = imul(value ^ (value >> 15U), value | 1U);
  value ^= value + imul(value ^ (value >> 7U), value | 61U);
  value ^= value >> 14U;
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
  const double theta = 2.0 * kPi * v;
  cached_gaussian_ = radius * std::sin(theta);
  has_cached_gaussian_ = true;
  return radius * std::cos(theta);
}

double gamma_lanczos(double z) {
  static const double coefficients[] = {
      676.5203681218851, -1259.1392167224028, 771.3234287776531,
      -176.6150291621406, 12.507343278686905, -0.13857109526572012,
      9.984369578019572e-6, 1.5056327351493116e-7};
  if (z < 0.5) return kPi / (std::sin(kPi * z) * gamma_lanczos(1.0 - z));

  double x = 0.9999999999998099;
  const double shifted = z - 1.0;
  for (int index = 0; index < 8; ++index) {
    x += coefficients[index] / (shifted + index + 1.0);
  }
  const double t = shifted + 7.5;
  return std::sqrt(2.0 * kPi) * std::pow(t, shifted + 0.5) * std::exp(-t) * x;
}

std::vector<double> levy_vector(double beta, int size, SeededRng& rng) {
  const double safe_beta = clamp(beta, 1.1, 1.99);
  const double numerator =
      gamma_lanczos(1.0 + safe_beta) * std::sin((kPi * safe_beta) / 2.0);
  const double denominator = gamma_lanczos((1.0 + safe_beta) / 2.0) * safe_beta *
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

}  // namespace tsp
