#pragma once
#include "graph.hpp"
#include "utils.hpp"

// low-level search node
struct Constraint {
  std::vector<int> who;
  Vertices where;
  const int depth;
  Constraint();
  Constraint(Constraint* parent, int i, Vertex* v);  // who and where
  ~Constraint();
};
