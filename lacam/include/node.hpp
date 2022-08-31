#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "utils.hpp"

// high-level search node
struct Node {
  const Config C;
  Node* parent;
  const int depth;

  // for low-level search
  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<Constraint*> search_tree;

  Node(Config _C, DistTable& D, Node* _parent = nullptr);
  Node(Config _C, DistTable& D, Node* _parent, std::vector<int>& agent_indexes);
  ~Node();
};
using Nodes = std::vector<Node*>;
