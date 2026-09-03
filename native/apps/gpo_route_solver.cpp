#include "gpo/route_solver_protocol.h"
#include "gpo/route_solver_service.h"

#include <exception>
#include <iostream>

int main() {
  try {
    const gpo::SolveConfig config = gpo::read_solve_config(std::cin);
    gpo::write_solve_success(std::cout, gpo::solve_route(config));
    return 0;
  } catch (const std::exception& error) {
    gpo::write_solve_error(std::cout, error.what());
    return 1;
  }
}
