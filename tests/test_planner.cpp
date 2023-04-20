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

TEST(planner, optimizer)
{
  const auto scen_filename = "./assets/loop.scen";
  const auto map_filename = "./assets/loop.map";
  const auto ins = Instance(scen_filename, map_filename, 3);
  auto additional_info = std::string();

  auto solution_m =
      solve(ins, additional_info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, solution_m));
  ASSERT_TRUE(get_makespan(solution_m) == 10);

  auto solution_l = solve(ins, additional_info, 0, nullptr, nullptr,
                          Objective::OBJ_SUM_OF_LOSS);
  ASSERT_TRUE(is_feasible_solution(ins, solution_l));
  ASSERT_TRUE(get_sum_of_loss(solution_l) == 15);
}
