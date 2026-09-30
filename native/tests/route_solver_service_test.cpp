#include "gpo/route_solver_service.h"

#include <cassert>

int main() {
  gpo::SolveConfig empty;
  empty.algorithm_key = "ga_tabu";
  const gpo::SolveOutput empty_output = gpo::solve_route(empty);
  assert(empty_output.route.empty());
  assert(!empty_output.closed);

  gpo::SolveConfig single;
  single.algorithm_key = "ga_tabu";
  single.task = gpo::TaskKind::kTsp;
  single.points.push_back({1.25, -2.5});
  const gpo::SolveOutput single_output = gpo::solve_route(single);
  assert(single_output.closed);
  assert(single_output.length == 0.0);
  assert((single_output.order == std::vector<int>{0}));
  assert(single_output.route.size() == 1U);
  assert(single_output.route[0].x == 1.25);
  for (const char* algorithm : {"ga_tabu", "otshig", "rasseivanie", "cuckoo"}) {
    gpo::SolveConfig pair;
    pair.algorithm_key = algorithm;
    pair.task = gpo::TaskKind::kTsp;
    pair.points = {{0, 0}, {1, 0}};
    const auto output = gpo::solve_route(pair);
    assert(output.closed && output.length == 2.0);
    assert((output.order == std::vector<int>{0, 1}));
    assert(output.route.size() == 3);
  }
}
