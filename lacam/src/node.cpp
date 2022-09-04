#include "../include/node.hpp"

#include <random>

// for high-level
Node::Node(Config _C, DistTable& D, Node* _parent, std::mt19937* MT)
    : C(_C),
      parent(_parent),
      depth(_parent == nullptr ? 0 : _parent->depth + 1),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<Constraint*>())
{
  search_tree.push(new Constraint());
  const auto N = C.size();

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (auto i = 0; i < N; ++i) {
      priorities[i] = std::make_tuple(0, 0, 0, (float)D.get(i, C[i]) / N);
    }

  } else {
    // dynamic priorities, akin to PIBT
    for (auto i = 0; i < N; ++i) {
      auto r = (int)(get_random_float(MT) < INFLATION_RATE);
      auto p =
          (D.get(i, C[i]) != 0) ? (std::get<1>(parent->priorities[i]) + 1) : 0;
      auto q =
          (D.get(i, C[i]) == 0) ? std::get<2>(parent->priorities[i]) - 1 : 0;
      priorities[i] =
          std::make_tuple(r, p, q, std::get<3>(parent->priorities[i]));
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int i, int j) { return priorities[i] > priorities[j]; });
}

Node::~Node()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}
