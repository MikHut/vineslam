#pragma once

#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/elevation_map.hpp>

namespace vineslam
{
class MapPostProcessor
{
public:
  // Class constructors
  MapPostProcessor() = default;
  MapPostProcessor(OccupancyMap* grid_map, ElevationMap* elevation_map);

  // Pedestrian filter
  void filterPedestrians();

private:
  OccupancyMap* m_grid_map;
  ElevationMap* m_elevation_map;
};

}  // namespace vineslam
