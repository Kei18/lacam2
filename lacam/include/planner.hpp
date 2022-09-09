#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "node.hpp"
#include "utils.hpp"

// PIBT agent
struct Agent {
  const uint id;
  Vertex* v_now;   // current location
  Vertex* v_next;  // next location
  Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using Agents = std::vector<Agent*>;

// next location candidates, for saving memory allocation
using Candidates = std::vector<std::array<Vertex*, 5> >;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;  // number of agents
  const uint V_size;
  DistTable D;
  Node* S_goal;                     // goal node
  Candidates C_next;                // used in PIBT
  std::vector<float> tie_breakers;  // random values, used in PIBT
  Agents A;
  Agents occupied_now;   // for quick collision checking
  Agents occupied_next;  // for quick collision checking

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0, const float _restart_rate = 0.001);
  Solution solve();
  void expand_lowlevel_tree(Node* S, Constraint* M);
  void update_cost(Node* S_from, Node* S_to);
  bool get_new_config(Node* S, Constraint* M);
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);

  // swap operations
  bool is_swap_required(uint id_h, uint id_l, Vertex* v_now_h, Vertex* v_now_l);
  bool is_pullable(Vertex* v_now, Vertex* v_opposite);
};

// main function
Solution solve(const Instance& ins, const int verbose = 0,
               const Deadline* deadline = nullptr, std::mt19937* MT = nullptr,
               const float restart_rate = 0.001);
