#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "utils.hpp"

// high-level search node
struct Node {
  const uint id;
  const Config C;
  Node* parent;
  std::unordered_map<uint, Node*> children;
  uint depth;

  // for low-level search
  std::vector<std::tuple<int, int, float>> priorities;
  std::vector<uint> order;
  std::queue<Constraint*> search_tree;

  Node(const Config& _C, DistTable& D, Node* _parent = nullptr);
  ~Node();
};
using Nodes = std::vector<Node*>;
