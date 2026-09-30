#pragma once

#include "tsp/local_search.h"

namespace tsp {

std::vector<Tour> build_initial_population(
    int size,
    const Problem& problem,
    SeededRng& rng,
    int local_moves);
const Tour& pick_best(const std::vector<Tour>& population);
const Tour& select_tournament(
    const std::vector<Tour>& population,
    SeededRng& rng,
    int size = 4);
std::vector<Tour> select_promising_seeds(const std::vector<Tour>& population, int count);
std::vector<Tour> build_ref_set(const std::vector<Tour>& population, int ref_size);

}  // namespace tsp
