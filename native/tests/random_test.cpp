#include "tsp/random.h"

#include <cassert>
#include <cmath>

int main() {
  tsp::SeededRng first(42U);
  tsp::SeededRng second(42U);
  for (int index = 0; index < 8; ++index) assert(first.uniform() == second.uniform());

  tsp::SeededRng rng(7U);
  for (int index = 0; index < 100; ++index) {
    const int value = rng.uniform_int(3, 5);
    assert(value >= 3 && value <= 5);
  }
  const std::vector<double> levy = tsp::levy_vector(1.5, 6, rng);
  assert(levy.size() == 6U);
  for (double value : levy) assert(std::isfinite(value));
}
