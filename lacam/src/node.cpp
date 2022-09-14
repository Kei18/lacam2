#include "../include/node.hpp"

uint Node::NODE_CNT = 0;

// for high-level
Node::Node(const Config& _C, DistTable& D, Node* _parent, const uint _g,
           const uint _h)
    : id(++NODE_CNT),
      C(_C),
      parent(_parent),
      neighbor(std::unordered_map<uint, Node*>()),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<Constraint*>())
{
  search_tree.push(new Constraint());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) {
    parent->neighbor[id] = this;
    neighbor[parent->id] = parent;
  }

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
