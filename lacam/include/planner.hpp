#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "node.hpp"
#include "utils.hpp"

// PIBT agent
struct Agent {
  const int id;    // high
  Vertex* v_now;   // current location
  Vertex* v_next;  // next location
  Agent(int _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using Agents = std::vector<Agent*>;

// next location candidates, for saving memory allocation
using Candidates = std::vector<std::array<Vertex*, 5> >;
static constexpr int NIL = -1;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTable D;

  // high-level
  std::vector<int> zones;
  std::vector<int> occupied;

  // low-level
  Agents A;
  std::vector<int> agent_indexes;
  std::vector<int> inverse_agent_indexes;
  Candidates C_next;
  std::vector<float> tie_breakers;  // random values, used in PIBT
  Agents occupied_now;              // for quick collision checking
  Agents occupied_next;             // for quick collision checking
  std::vector<int> constraints_who;
  Vertices constraints_where;

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          int _verbose = 0);
  Solution solve_high();
  Config get_new_config_high(Node* S, Constraint* M);

  Solution solve_low(Node* S_high, Constraint* M_high, const int zone);
  bool get_new_config_low(Node* S, Constraint* M, Constraint* M_high);
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);
};

// main function
Solution solve(const Instance& ins, const int verbose = 0,
               const Deadline* deadline = nullptr, std::mt19937* MT = nullptr);
