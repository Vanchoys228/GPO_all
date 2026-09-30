#pragma once

#include "tsp/tour_operations.h"

namespace tsp {

std::vector<int> nearest_neighbor_order(const Problem& problem);
Tour two_opt_improve(const std::vector<int>& order, const Problem& problem, int max_moves = 6);
std::vector<int> double_bridge_kick(const std::vector<int>& order, SeededRng& rng);

struct CandidateBuildOptions {
  int trials = 1;
  int move_strength = 3;
  int local_moves = 6;
  double kick_probability = 0.15;
};

Tour build_candidate_from_order(
    const std::vector<int>& base_order,
    const Problem& problem,
    SeededRng& rng,
    const CandidateBuildOptions& options);

}  // namespace tsp
