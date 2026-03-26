#pragma once

#include "tsp/common.h"

namespace tsp {

AlgorithmRun run_ga_tabu(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed);
AlgorithmRun run_annealing(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed);
AlgorithmRun run_scatter_search(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed);
AlgorithmRun run_cuckoo_search(const Problem& problem, const AlgorithmParams& params, std::uint32_t seed);

}  // namespace tsp
