#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "utils.hpp"

// high-level search node
struct Node {
  static uint NODE_CNT;
  const uint id;
  const Config C;
  // tree
  Node* parent;
  std::unordered_map<uint, Node*> neighbor;
  // costs
  uint g;
  const uint h;
  uint f;
  // low-level search
  std::vector<std::tuple<int, int, float>> priorities;
  std::vector<uint> order;
  std::queue<Constraint*> search_tree;

  Node(const Config& _C, DistTable& D, Node* _parent, const uint _g,
       const uint _h);
  ~Node();
};
using Nodes = std::vector<Node*>;
