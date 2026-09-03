#pragma once

#include "tsp/random.h"

#include <vector>

namespace tsp {

int route_length(const std::vector<int>& order, const Problem& problem);
Tour clone_tour(const Tour& tour);
void reverse_segment(std::vector<int>& order, int start, int end);
void swap_positions(std::vector<int>& order, int left, int right);
std::vector<int> random_order(int size, SeededRng& rng);
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

}  // namespace tsp
