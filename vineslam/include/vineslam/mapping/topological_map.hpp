#pragma once

#include <vector>

#include <vineslam/mapping/occupancy_map.hpp>

namespace vineslam
{

struct Node
{

};

class TopologicalMap
{
public:
  TopologicalMap() = default;

private:
  std::vector<Node> m_nodes;
};
}  // namespace vineslam