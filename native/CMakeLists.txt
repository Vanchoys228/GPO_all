cmake_minimum_required(VERSION 3.20)

project(gpo_route_solver LANGUAGES CXX)

add_executable(
  gpo_route_solver
  apps/gpo_route_solver.cpp
  src/annealing.cpp
  src/common.cpp
  src/cuckoo.cpp
  src/ga_tabu.cpp
  src/scatter.cpp
)

target_include_directories(gpo_route_solver PRIVATE include)
target_compile_features(gpo_route_solver PRIVATE cxx_std_20)

if(MSVC)
  target_compile_options(gpo_route_solver PRIVATE /O2 /W4)
else()
  target_compile_options(gpo_route_solver PRIVATE -O2 -Wall -Wextra -Wpedantic)
endif()
