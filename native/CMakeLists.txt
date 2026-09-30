cmake_minimum_required(VERSION 3.20)

project(gpo_route_solver LANGUAGES CXX)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/route_solver_sources.txt" ROUTE_SOLVER_CORE_SOURCES)

add_library(gpo_route_solver_core STATIC ${ROUTE_SOLVER_CORE_SOURCES})
add_executable(gpo_route_solver apps/gpo_route_solver.cpp)
target_link_libraries(gpo_route_solver PRIVATE gpo_route_solver_core)

target_include_directories(gpo_route_solver_core PUBLIC include)
target_compile_features(gpo_route_solver_core PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(gpo_route_solver_core PRIVATE /O2 /W4)
else()
  target_compile_options(gpo_route_solver_core PRIVATE -O2 -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  foreach(test_name route_solver_protocol_test route_problem_test route_solver_service_test random_test tour_operations_test local_search_test population_test)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE gpo_route_solver_core)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
