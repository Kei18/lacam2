#include "../include/node.hpp"

static uint NODE_ID = 0;

// for high-level
Node::Node(const Config& _C, DistTable& D, Node* _parent)
    : id(++NODE_ID),
      C(_C),
      parent(_parent),
      children(std::unordered_map<uint, Node*>()),
      depth(_parent == nullptr ? 0 : _parent->depth + 1),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<Constraint*>())
{
  search_tree.push(new Constraint());
  const auto N = C.size();

  // update children of parent
  if (parent != nullptr) parent->children[id] = this;

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (uint i = 0; i < N; ++i)
      priorities[i] = {0, 0, (float)D.get(i, C[i]) / N};
  } else {
    // dynamic priorities, akin to PIBT
    for (uint i = 0; i < N; ++i) {
      priorities[i] = {
          (D.get(i, C[i]) != 0) ? std::get<0>(parent->priorities[i]) + 1 : 0,
          (D.get(i, C[i]) == 0) ? std::get<1>(parent->priorities[i]) - 1 : 0,
          std::get<2>(parent->priorities[i])};
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; });
}

Node::~Node()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}
