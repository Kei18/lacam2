/*
 * low-level search node
 */

#pragma once
#include "graph.hpp"
#include "utils.hpp"

struct Constraint {
  std::vector<uint> who;
  Vertices where;
  const uint depth;
  Constraint();
  Constraint(Constraint* parent, uint i, Vertex* v);  // who and where
  ~Constraint();
};
