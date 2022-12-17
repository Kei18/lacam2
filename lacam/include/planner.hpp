/*
 * lacam-star
 */

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

// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;
  Node* S_goal;                     // auxiliary, goal node
  uint loop_cnt;                    // auxiliary
  Candidates C_next;                // used in PIBT
  std::vector<float> tie_breakers;  // random values, used in PIBT
  Agents A;
  Agents occupied_now;   // for quick collision checking
  Agents occupied_next;  // for quick collision checking

  // for logging
  std::vector<int> hist_cost;
  std::vector<int> hist_time;

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001);
  ~Planner();
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(Node* S, Constraint* M);
  void rewrite(Node* S, Node* T);
  uint get_edge_cost(const Config& C1, const Config& C2);
  uint get_edge_cost(Node* S, Node* T);
  uint get_h_value(const Config& C);
  bool get_new_config(Node* S, Constraint* M);
  bool funcPIBT(Agent* ai, Agent* aj = nullptr);

  // swap operation
  bool is_swap_required(uint id_h, uint id_l, Vertex* v_now_h, Vertex* v_now_l);
  bool is_pullable(Vertex* v_now, Vertex* v_opposite);

  // utilities
  void update_hist();
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << Node::NODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};

// main function
Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose = 0, const Deadline* deadline = nullptr,
               std::mt19937* MT = nullptr, const Objective objective = OBJ_NONE,
               const float restart_rate = 0.001);
