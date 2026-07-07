#pragma once

#include <vector>
#include <Eigen/Dense>

struct GridMap
{
  int width;
  int height;
  double resolution;

  double x_min, y_min;

  std::vector<int8_t> obstacle;
  std::vector<int8_t> collision;

  GridMap(int w, int h, double res,
          double x0, double y0)
  : width(w), height(h),
    resolution(res),
    x_min(x0), y_min(y0)
  {
    obstacle.resize(w * h, 0);
    collision.resize(w * h, 0);
  }

  inline int index(int x, int y) const
  {
    return y * width + x;
  }

  inline bool inBounds(int x, int y) const
  {
    return x >= 0 && x < width &&
           y >= 0 && y < height;
  }

  void worldToIndex(
    const Eigen::Vector3d& pos,
    int& x, int& y) const
  {
    x = static_cast<int>(
      std::floor((pos.x() - x_min) / resolution));
    y = static_cast<int>(
      std::floor((pos.y() - y_min) / resolution));
  }
};