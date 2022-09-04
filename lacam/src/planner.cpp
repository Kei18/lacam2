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
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
}

Solution Planner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;
  std::vector<Constraint*> GC;  // garbage collection of constraints

  // insert initial node
  auto S = new Node(ins->starts, D);
  OPEN.push(S);
  CLOSED[S->C] = S;

  // best first search
  int loop_cnt = 0;
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    S = OPEN.top();
    info(2, verbose, "elapsed:", elapsed_ms(deadline), "ms",
         "\titer:", loop_cnt, "\tdepth:", S->depth, "\tconfig:", S->C);

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
    if (!get_new_config(S, M)) continue;

    // create new configuration
    auto C = Config(N, nullptr);
    for (auto a : A) C[a->id] = a->v_next;

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
  for (auto a : A) delete a;
  for (auto M : GC) delete M;
  for (auto p : CLOSED) delete p.second;

  return solution;
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
  for (auto k = 0; k < M->depth; ++k) {
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
  auto swap_operation = [&](const int k) {
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
  };
  // for clear operation
  if (aj != nullptr && C_next[i][0] != ai->v_now && C_next[i][0] != aj->v_now) {
    if (is_swap_required(aj->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_pullable(ai->v_now, C_next[i][0])) {
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
    }
  }

  // main operation
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

bool Planner::is_swap_required(int id_h, int id_l, Vertex* v_now_h,
                               Vertex* v_now_l)
{
  // simulating push
  auto v_h = v_now_h;
  auto v_l = v_now_l;
  while (D.get(id_h, v_l) < D.get(id_h, v_h)) {
    // corridor?
    auto n = v_l->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_l->neighbor) {
      auto a = occupied_now[u->id];
      if (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)
        --n;
    }
    if (n <= 1) {
      break;
    } else if (n >= 3) {
      return false;
    } else {
      auto tmp = v_h;
      v_h = v_l;
      v_l = (v_l->neighbor[0] == tmp) ? v_l->neighbor[1] : v_l->neighbor[0];
    }
  }
  return (D.get(id_l, v_h) < D.get(id_l, v_l)) &&
         (D.get(id_h, v_h) == 0 || D.get(id_h, v_l) < D.get(id_h, v_h));
}

bool Planner::is_pullable(Vertex* v_now, Vertex* v_opposite)
{
  auto v_pre = v_opposite;
  auto v_next = v_now;
  while (v_next->neighbor.size() == 2) {
    auto tmp = v_next;
    v_next = v_next->neighbor[(v_next->neighbor[0] == v_pre) ? 1 : 0];
    v_pre = tmp;
  }
  return v_next->neighbor.size() != 1;
}

Solution solve(const Instance& ins, const int verbose, const Deadline* deadline,
               std::mt19937* MT)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
  auto planner = Planner(&ins, deadline, MT, verbose);
  return planner.solve();
}
