#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "utils.hpp"

// high-level search node
struct Node {
  const Config C;
  Node* parent;
  const uint depth;

  // for low-level search
  std::vector<std::tuple<int, int, float>> priorities;
  std::vector<uint> order;
  std::queue<Constraint*> search_tree;

  Node(const Config& _C, DistTable& D, Node* _parent = nullptr);
  ~Node();
};
using Nodes = std::vector<Node*>;
