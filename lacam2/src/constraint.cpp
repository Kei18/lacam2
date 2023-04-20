#include "../include/constraint.hpp"

LNode::LNode() : who(std::vector<uint>()), where(Vertices()), depth(0) {}

LNode::LNode(LNode* parent, uint i, Vertex* v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
}

LNode::~LNode(){};
