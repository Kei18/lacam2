#include <lacam.hpp>

#include "gtest/gtest.h"

TEST(planner, solve)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);
  auto additional_info = std::string();

  auto solution = solve(ins, additional_info);
  ASSERT_TRUE(is_feasible_solution(ins, solution));
}
