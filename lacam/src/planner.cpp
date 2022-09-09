#include "../include/planner.hpp"

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const float _restart_rate)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      S_goal(nullptr),
      loop_cnt(0),
      node_cnt(0),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
  // setup agents
  for (uint i = 0; i < N; ++i) A[i] = new Agent(i);
}

Planner::~Planner()
{
  for (auto a : A) delete a;
}

Solution Planner::solve()
{
  solver_info(1, "start search");

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;

  // insert initial node
  auto S_init = new Node(ins->starts, D);
  OPEN.push(S_init);
  ++node_cnt;
  CLOSED[S_init->C] = S_init;

  // BFS
  std::vector<Config> solution;
  auto C_new = Config(N, nullptr);  // new configuration

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    auto S = OPEN.top();

    // low-level search end
    if (S->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // check goal condition
    if (is_same_config(S->C, ins->goals)) {
      if (S_goal == nullptr) {
        S_goal = S;
        solver_info(1, "found solution, cost: ", S->depth);
      }
      // random insert
      auto S_rand = std::next(std::begin(CLOSED),
                              get_random_int(MT, 0, CLOSED.size() - 1))
                        ->second;
      OPEN.push(S_rand);
      continue;
    }

    // create successors at the low-level search
    auto M = S->search_tree.front();
    S->search_tree.pop();
    expand_lowlevel_tree(S, M);

    // create successors at the high-level search
    const auto res = get_new_config(S, M);
    delete M;  // free
    if (res) {
      // create new configuration
      for (auto a : A) C_new[a->id] = a->v_next;

      // check explored list
      const auto iter = CLOSED.find(C_new);
      if (iter != CLOSED.end()) {
        // case found
        update_cost(S, iter->second);
        // re-insert or random-restart
        OPEN.push(get_random_float(MT) >= RESTART_RATE ? iter->second : S_init);
      } else {
        // insert new search node
        const auto S_new = new Node(C_new, D, S);
        OPEN.push(S_new);
        ++node_cnt;
        CLOSED[S_new->C] = S_new;
      }
    }
  }

  // backtrack
  if (S_goal != nullptr) {
    auto S = S_goal;
    while (S != nullptr) {
      solution.push_back(S->C);
      S = S->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  if (S_goal != nullptr) {
    solver_info(1, "solved");
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else {
    solver_info(1, "timeout");
  }

  // memory management
  for (auto p : CLOSED) delete p.second;

  return solution;
}

void Planner::expand_lowlevel_tree(Node* S, Constraint* M)
{
  if (M->depth >= N) return;
  const auto i = S->order[M->depth];
  auto C = S->C[i]->neighbor;
  C.push_back(S->C[i]);
  // randomize
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  // insert
  for (auto v : C) S->search_tree.push(new Constraint(M, i, v));
}

void Planner::update_cost(Node* S_from, Node* S_to)
{
  if (S_to->depth <= S_from->depth + 1) return;

  // update tree structure
  S_to->parent->children.erase(S_to->id);
  S_to->parent = S_from;
  S_from->children[S_to->id] = S_to;
  // update children costs using BFS
  std::queue<Node*> Q;
  Q.push(S_to);
  while (!Q.empty()) {
    auto S = Q.front();
    Q.pop();
    if (S == S_goal)
      solver_info(1, "cost update: ", S->depth, " -> ", S->parent->depth + 1);
    S->depth = S->parent->depth + 1;
    for (auto iter : S->children) Q.push(iter.second);
  }
}

bool Planner::get_new_config(Node* S, Constraint* M)
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

  // add constraints
  for (uint k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];        // agent
    const auto l = M->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = S->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
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
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  // for swap situation
  Agent* swap_agent = nullptr;
  {
    auto al = occupied_now[C_next[i][0]->id];
    if (al != nullptr && al != ai && al->v_next == nullptr &&
        is_swap_required(ai->id, al->id, ai->v_now, al->v_now) &&
        is_pullable(ai->v_now, al->v_now)) {
      swap_agent = al;
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
    }
  }
  auto swap_operation = [&](const uint k) {
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      // pull swap_agent
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
  };
  // for clear operation
  if (swap_agent == nullptr) {
    for (auto u : ai->v_now->neighbor) {
      auto ah = occupied_now[u->id];
      if (ah == nullptr || C_next[i][0] == ai->v_now ||
          C_next[i][0] == ah->v_now)
        continue;
      if (is_swap_required(ah->id, ai->id, ai->v_now, C_next[i][0]) &&
          is_pullable(ai->v_now, C_next[i][0])) {
        std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
        break;
      }
    }
  }

  // main operation
  for (size_t k = 0; k < K + 1; ++k) {
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
    if (ak == nullptr || u == ai->v_now) {
      swap_operation(k);
      return true;
    }

    // priority inheritance
    if (ak->v_next == nullptr && !funcPIBT(ak, ai)) continue;

    // success to plan next one step
    swap_operation(k);
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

bool Planner::is_swap_required(uint id_h, uint id_l, Vertex* v_now_h,
                               Vertex* v_now_l)
{
  // simulating push
  auto v_h = v_now_h;
  auto v_l = v_now_l;
  Vertex* tmp = nullptr;
  while (D.get(id_h, v_l) < D.get(id_h, v_h)) {
    auto n = v_l->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_l->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_h ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap at v_l
    if (n <= 0) break;
    v_h = v_l;
    v_l = tmp;
  }

  return (D.get(id_l, v_h) < D.get(id_l, v_l)) &&
         (D.get(id_h, v_h) == 0 || D.get(id_h, v_l) < D.get(id_h, v_h));
}

bool Planner::is_pullable(Vertex* v_now, Vertex* v_opposite)
{
  // simulate pull
  auto v_pre = v_opposite;
  auto v_next = v_now;
  Vertex* tmp = nullptr;
  while (true) {
    auto n = v_next->neighbor.size();
    for (auto u : v_next->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pre ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;  // able to swap at v_next
    if (n <= 0) return false;
    v_pre = v_next;
    v_next = tmp;
  }
  return false;
}

Solution solve(const Instance& ins, const int verbose, const Deadline* deadline,
               std::mt19937* MT, const float restart_rate)
{
  auto planner = Planner(&ins, deadline, MT, verbose, restart_rate);
  return planner.solve();
}
