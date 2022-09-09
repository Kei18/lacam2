#include "../include/constraint.hpp"

Constraint::Constraint() : who(std::vector<uint>()), where(Vertices()), depth(0)
{
}

Constraint::Constraint(Constraint* parent, uint i, Vertex* v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
}

Constraint::~Constraint(){};
