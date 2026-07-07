#pragma once

#include <string>
#include <cmath>

struct GridMapConfig
{
  std::string frame_id = "base_link";

  float resolution = 0.03;

  // 地图范围（世界坐标）
  float x_min = 0.0f;
  float x_max = 10.0f;
  float y_min = -5.0f;
  float y_max = 5.0f;

  // 栅格数量（自动算）
  int width_index() const
  {
    return static_cast<int>(
      std::ceil((x_max - x_min) / resolution));
  }

  int length_index() const
  {
    return static_cast<int>(
      std::ceil((y_max - y_min) / resolution));
  }
};