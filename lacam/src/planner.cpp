#include "../include/planner.hpp"

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, int _verbose)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      zones(N, NIL),
      occupied(V_size, NIL),
      inverse_agent_indexes(N, NIL),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
}

Solution Planner::solve_high()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;
  std::vector<Constraint*> GC;  // garbage collection of constraints

  // insert initial node
  auto S = new Node(ins->starts, D);
  OPEN.push(S);
  CLOSED[S->C] = S;

  // depth first search
  int loop_cnt = 0;
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    S = OPEN.top();

    // check goal condition
    if (is_same_config(S->C, ins->goals)) {
      // backtrack
      while (S != nullptr) {
        solution.push_back(S->C);
        S = S->parent;
      }
      std::reverse(solution.begin(), solution.end());
      break;
    }

    // low-level search end
    if (S->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // create successors at the low-level search
    auto M = S->search_tree.front();
    GC.push_back(M);
    S->search_tree.pop();
    if (M->depth < N) {
      auto i = S->order[M->depth];
      auto C = S->C[i]->neighbor;
      C.push_back(S->C[i]);
      if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
      for (auto u : C) S->search_tree.push(new Constraint(M, i, u));
    }

    // create successors at the high-level search
    auto C = get_new_config_high(S, M);
    if (C.empty()) break;
    println(C);

    // check explored list
    auto iter = CLOSED.find(C);
    if (iter != CLOSED.end()) {
      OPEN.push(iter->second);
      continue;
    }

    // insert new search node
    auto S_new = new Node(C, D, S);
    OPEN.push(S_new);
    CLOSED[S_new->C] = S_new;
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? "failed" : "solution found", "\texpanded:", loop_cnt,
       "\texplored:", CLOSED.size());
  // memory management
  for (auto M : GC) delete M;
  for (auto p : CLOSED) delete p.second;

  return solution;
}

Config Planner::get_new_config_high(Node* S, Constraint* M)
{
  int zone = -1;
  std::fill(zones.begin(), zones.end(), NIL);

  // identify independent sets
  for (auto i = 0; i < N; ++i) occupied[S->C[i]->id] = i;
  for (auto i = 0; i < N; ++i) {
    if (zones[i] != NIL) continue;
    zone += 1;
    zones[i] = zone;
    auto OPEN = std::queue<int>();
    OPEN.push(i);
    while (!OPEN.empty()) {
      auto j = OPEN.front();
      OPEN.pop();
      // one-hop
      for (auto v : S->C[j]->neighbor) {
        auto k = occupied[v->id];
        if (k != NIL && zones[k] == NIL) {
          zones[k] = zone;
          OPEN.push(k);
        }
        // two-hop
        for (auto u : v->neighbor) {
          k = occupied[u->id];
          if (k != NIL && zones[k] == NIL) {
            zones[k] = zone;
            OPEN.push(k);
          }
        }
      }
    }
  }

  // apply low-level planner
  auto C_new = Config(N, nullptr);
  for (auto k = 0; k <= zone; ++k) {
    auto sub_solution = solve_low(S, M, k);
    if (sub_solution.empty()) return Config();
    println(sub_solution);
    for (auto i = 0; i < agent_indexes.size(); ++i) {
      C_new[agent_indexes[i]] =
          sub_solution[sub_solution.size() > 1 ? 1 : 0][i];
    }
  }
  std::cout << std::endl;

  // cleanup
  for (auto i = 0; i < N; ++i) occupied[S->C[i]->id] = NIL;

  return C_new;
}

Solution Planner::solve_low(Node* S_high, Constraint* M_high, const int zone)
{
  // setup instance & agents
  Config starts, goals;
  A.clear();
  agent_indexes.clear();
  constraints_who.clear();
  constraints_where.clear();

  for (auto i = 0; i < N; ++i) {
    if (zones[i] != zone) {
      inverse_agent_indexes[i] = NIL;
      continue;
    }
    auto j = A.size();
    A.push_back(new Agent(j));
    starts.push_back(S_high->C[i]);
    goals.push_back(ins->goals[i]);
    agent_indexes.push_back(i);
    inverse_agent_indexes[i] = j;
  }
  for (auto k = 0; k < M_high->depth; ++k) {
    auto i = inverse_agent_indexes[M_high->who[k]];
    if (i != NIL) {
      constraints_who.push_back(i);
      constraints_where.push_back(M_high->where[k]);
    }
  }

  const auto N_sub = agent_indexes.size();

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;
  std::vector<Constraint*> GC;  // garbage collection of constraints

  // insert initial node
  auto S = new Node(starts, D, nullptr, agent_indexes);
  OPEN.push(S);
  CLOSED[S->C] = S;

  // depth first search
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    S = OPEN.top();
    if (is_same_config(S->C, goals)) {
      while (S != nullptr) {
        solution.push_back(S->C);
        S = S->parent;
      }
      std::reverse(solution.begin(), solution.end());
      break;
    }
    if (S->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // create successors at the low-level search
    auto M = S->search_tree.front();
    GC.push_back(M);
    S->search_tree.pop();
    if (M->depth < S->C.size()) {
      auto i = S->order[M->depth];
      auto C = S->C[i]->neighbor;  // candidates
      C.push_back(S->C[i]);
      if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
      for (auto u : C) S->search_tree.push(new Constraint(M, i, u));
    }

    // create successors at the high-level search
    if (!get_new_config_low(S, M, M_high)) continue;

    // create new configuration
    auto C = Config(N_sub, nullptr);
    for (auto i = 0; i < N_sub; ++i) C[i] = A[i]->v_next;

    // check explored list
    auto iter = CLOSED.find(C);
    if (iter != CLOSED.end()) {
      OPEN.push(iter->second);
      continue;
    }

    // insert new search node
    auto S_new = new Node(C, D, S, agent_indexes);
    OPEN.push(S_new);
    CLOSED[S_new->C] = S_new;
  }
  // cleanup
  for (auto a : A) {
    if (a->v_now != nullptr) occupied_now[a->v_now->id] = nullptr;
    if (a->v_next != nullptr) occupied_next[a->v_next->id] = nullptr;
  }
  // memory management
  for (auto a : A) delete a;
  for (auto M : GC) delete M;
  for (auto p : CLOSED) delete p.second;
  return solution;
}

bool Planner::get_new_config_low(Node* S, Constraint* M, Constraint* M_high)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }
    // set occupied now
    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints from M_high
  if (S->depth == 0) {
    for (auto k = 0; k < constraints_who.size(); ++k) {
      const auto i = constraints_who[k];        // agent
      const auto l = constraints_where[k]->id;  // loc

      std::cout << "who:" << i << ", where:" << l << std::endl;

      // check vertex collision
      if (occupied_next[l] != nullptr) return false;

      // check swap collision
      auto l_pre = S->C[i]->id;
      if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
          occupied_next[l_pre]->id == occupied_now[l]->id)
        return false;

      // set occupied_next
      A[i]->v_next = constraints_where[k];
      occupied_next[l] = A[i];
    }
  }

  // add constraints from M
  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];        // agent
    const auto l = M->where[k]->id;  // loc

    // check consistency with M_high
    if (A[i]->v_next != nullptr && A[i]->v_next->id != l) return false;

    // check vertex collision
    if (occupied_next[l] != nullptr && occupied_next[l] != A[i]) return false;
    // check swap collision
    auto l_pre = S->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id &&
        occupied_now[l]->id != i)
      return false;

    // set occupied_next
    A[i]->v_next = M->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent* ai, Agent* aj)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(agent_indexes[i], v) + tie_breakers[v->id] <
                     D.get(agent_indexes[i], u) + tie_breakers[u->id];
            });

  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;
    // avoid swap conflicts
    if (aj != nullptr && u == aj->v_now) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap confilicts with constraints
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // empty or stay
    if (ak == nullptr || u == ai->v_now) return true;

    // priority inheritance
    if (ak->v_next == nullptr && !funcPIBT(ak, ai)) continue;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Solution solve(const Instance& ins, const int verbose, const Deadline* deadline,
               std::mt19937* MT)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
  auto planner = Planner(&ins, deadline, MT, verbose);
  return planner.solve_high();
}
