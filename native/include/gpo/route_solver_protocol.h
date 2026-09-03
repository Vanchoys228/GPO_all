#pragma once

#include "gpo/route_solver_types.h"

#include <iosfwd>
#include <string>

namespace gpo {

SolveConfig read_solve_config(std::istream& input);
void write_solve_success(std::ostream& output, const SolveOutput& result);
void write_solve_error(std::ostream& output, const std::string& message);

}  // namespace gpo
