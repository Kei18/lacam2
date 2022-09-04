#pragma once

#include <random>

#include "constraint.hpp"
#include "dist_table.hpp"
#include "utils.hpp"

static constexpr float INFLATION_RATE = 0.001;

// high-level search node
struct Node {
  const Config C;
  Node* parent;
  const int depth;

  // for low-level search
  std::vector<std::tuple<int, int, int, float>> priorities;
  std::vector<int> order;
  std::queue<Constraint*> search_tree;

  Node(Config _C, DistTable& D, Node* _parent = nullptr,
       std::mt19937* MT = nullptr);
  ~Node();
};
using Nodes = std::vector<Node*>;
