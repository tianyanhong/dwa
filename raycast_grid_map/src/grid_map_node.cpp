#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include "raycast_grid_map/grid_map_config.h"
#include "raycast_grid_map/grid_map.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class GridMapGenerator
{
public:
  GridMapGenerator()
  : nh_("~")
  {
    config_ = GridMapConfig();

    grid_map_ = std::make_unique<GridMap>(
      config_.width_index(),
      config_.length_index(),
      config_.resolution,
      config_.x_min,
      config_.y_min
    );
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>( "/no_ground_pc1", 1 , &GridMapGenerator::pointCloudCallback, this);
    path_sub_ = nh_.subscribe("/path",1,&GridMapGenerator::pathCallback,this);
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
    timer_ = nh_.createTimer(
      ros::Duration(0.1),
      &GridMapGenerator::callback, this);
  }

private:
    void publish()
    {

        nav_msgs::OccupancyGrid msg;
        msg.header.frame_id = config_.frame_id;
        msg.header.stamp = ros::Time::now();

        msg.info.resolution = config_.resolution;
        msg.info.width = grid_map_->width;
        msg.info.height = grid_map_->height;

        msg.info.origin.position.x = config_.x_min;
        msg.info.origin.position.y = config_.y_min;
        msg.info.origin.orientation.w = 1.0;

        msg.data = grid_map_->collision;

        pub_.publish(msg);
    }
    void callback(const ros::TimerEvent&)
    {
        buildMap();
        publish();

    }
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        latest_cloud_ = *msg;
        has_cloud_ = true;
    }
    void pathCallback(const nav_msgs::PathConstPtr& msg)
    {
        latest_path_ = *msg;
        has_path_ = true;
    }
    void buildMap()
    {
        grid_map_->obstacle.assign(
            grid_map_->width * grid_map_->height, 0);
        grid_map_->collision.assign(
            grid_map_->width * grid_map_->height, 0);

        if (!has_cloud_)
            return;

        sensor_msgs::PointCloud2ConstIterator<float>
            iter_x(latest_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float>
            iter_y(latest_cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float>
            iter_z(latest_cloud_, "z");

        for (; iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            if (std::isnan(x) || std::isnan(y))
            continue;

            Eigen::Vector3d pos(x, y, z);

            addObstacle(pos, 0);
            addCollision(pos, 0.4);
        }
    }

    void addObstacle(const Eigen::Vector3d& pos, double radius)
    {
        int ix, iy;
        grid_map_->worldToIndex(pos, ix, iy);

        int r = static_cast<int>(radius / config_.resolution);

        for (int dx = -r; dx <= r; ++dx)
        {
        for (int dy = -r; dy <= r; ++dy)
        {
            int x = ix + dx;
            int y = iy + dy;
            if (!grid_map_->inBounds(x, y)) continue;

            if (dx*dx + dy*dy <= r*r)
            grid_map_->obstacle[grid_map_->index(x, y)] = 100;
        }
        }
    }

    void addCollision(const Eigen::Vector3d& pos, double radius)
    {
        int ix, iy;
        grid_map_->worldToIndex(pos, ix, iy);

        int r = static_cast<int>(radius / config_.resolution);

        for (int dx = -r; dx <= r; ++dx)
        {
        for (int dy = -r; dy <= r; ++dy)
        {
            int x = ix + dx;
            int y = iy + dy;
            if (!grid_map_->inBounds(x, y)) continue;

            if (dx*dx + dy*dy <= r*r)
            grid_map_->collision[grid_map_->index(x, y)] = 100;
        }
        }
    }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;
ros::Subscriber path_sub_;
nav_msgs::Path latest_path_;
bool has_path_ = false;
sensor_msgs::PointCloud2 latest_cloud_;
bool has_cloud_ = false;

  GridMapConfig config_;
  std::unique_ptr<GridMap> grid_map_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_generator");
  GridMapGenerator gen;
  ros::spin();
  return 0;
}