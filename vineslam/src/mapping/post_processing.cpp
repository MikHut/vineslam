#include <vineslam/mapping/post_processing.hpp>

namespace vineslam
{
MapPostProcessor::MapPostProcessor(OccupancyMap* grid_map, ElevationMap* elevation_map)
{
  m_grid_map = grid_map;
  m_elevation_map = elevation_map;
}

void MapPostProcessor::filterPedestrians()
{
}

}  // namespace vineslam