#include "raycast_grid_map/ray_tracer.h"
#include <cmath>

RayTracer::RayTracer(GridMap& map, float max_range)
  : grid_map_(map), max_range_(max_range) {}

void RayTracer::trace(const geometry_msgs::Point& origin,
                      const geometry_msgs::Point& end) {
  float dx = end.x - origin.x;
  float dy = end.y - origin.y;
  float distance = std::sqrt(dx * dx + dy * dy);

  if (distance > max_range_) return;

  int steps = static_cast<int>(distance / 0.05f);
  for (int i = 0; i < steps; ++i) {
    float t = static_cast<float>(i) / steps;
    float x = origin.x + dx * t;
    float y = origin.y + dy * t;

    // grid_map_.setOccupied(x, y);
  }
}