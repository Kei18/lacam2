/*
 * high-level search node
 */

#pragma once

#include "constraint.hpp"
#include "dist_table.hpp"
#include "utils.hpp"

// high-level node
struct HNode {
  static uint HNODE_CNT;  // for id
  const uint id;          // used in neighbor
  const Config C;
  // tree
  HNode* parent;
  std::unordered_map<uint, HNode*> neighbor;
  // costs
  uint g;
  const uint h;
  uint f;
  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();
};
using HNodes = std::vector<HNode*>;
