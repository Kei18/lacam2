/*
 * low-level search node
 */

#pragma once
#include "graph.hpp"
#include "utils.hpp"

// low-level node
struct LNode {
  std::vector<uint> who;
  Vertices where;
  const uint depth;
  LNode();
  LNode(LNode* parent, uint i, Vertex* v);  // who and where
  ~LNode();
};
