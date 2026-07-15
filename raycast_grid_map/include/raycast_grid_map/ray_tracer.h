#pragma once

#include "raycast_grid_map/grid_map.h"
#include <geometry_msgs/Point.h>

class RayTracer {
    public:
        RayTracer(GridMap& map, float max_range);

        void trace(const geometry_msgs::Point& origin,
                    const geometry_msgs::Point& end);

    private:
        GridMap& grid_map_;
        float max_range_;
};