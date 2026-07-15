#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


void pub_odom(ros::Publisher &odom_pub,tf::TransformBroadcaster &odom_broadcaster)
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double vx = 0.5;   // m/s
    double vy = 0.0;
    double vth = 0.1;  // rad/s

    // 简单运动模型（用于测试）
    double dt = 1.0 / 20.0;
    theta += vth * dt;
    x += (vx * cos(theta) - vy * sin(theta)) * dt;
    y += (vx * sin(theta) + vy * cos(theta)) * dt;

    // ====== Odometry Message ======
    nav_msgs::Odometry odom;
    ros::Time current_time;
    current_time = ros::Time::now();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);

    // ====== TF ======

    tf::Transform transform2;
    transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0, 0, 0);
    transform2.setRotation(q);
    odom_broadcaster.sendTransform(
        tf::StampedTransform(transform2, current_time, "map", "odom"));
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(q);
    odom_broadcaster.sendTransform(
        tf::StampedTransform(transform, current_time, "odom", "base_link"));

    tf::Transform transform3;
    transform2.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
    q.setRPY(0, 0, 0);
    transform2.setRotation(q);
    odom_broadcaster.sendTransform(
        tf::StampedTransform(transform, current_time, "base_link", "scan"));

}

void pub_msg(ros::Publisher pub_dis)
{
    double threshold = 0.1; // 默认阈值（米） 0.5---3.0s
    std_msgs::Float64 msg;
    msg.data = threshold;

    pub_dis.publish(msg);
}

void pub_footprint(ros::Publisher &pub_footprint_msg)
{
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    footprint.header.stamp = ros::Time::now();

    // 逆时针定义多边形
    float length = 1.5,width = 0.8;
    geometry_msgs::Point32 p;
    p.x =  length / 2.0;  p.y =  width / 2.0;
    footprint.polygon.points.push_back(p);

    p.x = -length / 2.0; p.y =  width / 2.0;
    footprint.polygon.points.push_back(p);

    p.x = -length / 2.0; p.y = -width / 2.0;
    footprint.polygon.points.push_back(p);

    p.x =  length / 2.0; p.y = -width / 2.0;
    footprint.polygon.points.push_back(p);

    pub_footprint_msg.publish(footprint);

}
void goal_pub_function(ros::Publisher &goal_pub_msg)
{
  geometry_msgs::PoseStamped goal;

  // ===== 目标位姿 =====
  goal.header.frame_id = "map";   // 或 "odom"
  goal.header.stamp = ros::Time::now();

  goal.pose.position.x = 5.0;
  goal.pose.position.y = 0.0;
  goal.pose.position.z = 0.0;

  goal.pose.orientation.w = 1.0;  // 朝向 0°
  goal_pub_msg.publish(goal);

}

void pub_path_function(ros::Publisher &path_pub)
{
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  // 构造一条简单直线路径
  for (int i = 0; i <= 100; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub.publish(path);
}

void pub_scan(ros::Publisher &pub_scan_msg)
{
  sensor_msgs::LaserScan scan;

  scan.header.frame_id = "base_link"; // 或 laser
  scan.angle_min = -M_PI;
  scan.angle_max =  M_PI;
  scan.angle_increment = M_PI / 180.0; // 1°
  scan.time_increment = 0.0;
  scan.scan_time = 0.1;
  scan.range_min = 0.1;
  scan.range_max = 20.0;
  const int num = 361;
  scan.ranges.resize(num, scan.range_max);
  scan.intensities.resize(num, 0);

    // ✅ 正前方 5m 处障碍物
    int front_index = 185;
    for(int i= front_index;i < (front_index + 100);i++)
    {
        scan.ranges[i] = 1.5; 
    }

    int front_index_2 = 140;
    for(int i= front_index_2;i < (front_index_2 + 50);i++)
    {
        scan.ranges[i] = 3.0; 
    }
  scan.header.stamp = ros::Time::now();
  pub_scan_msg.publish(scan);

}

auto makeField = [](const std::string& name, uint32_t offset) {
    sensor_msgs::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    return f;
};

sensor_msgs::PointCloud2 toPointCloud2(
    const std::vector<Eigen::Vector3f>& pts,
    const std::string& frame_id)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = ros::Time::now();;
    cloud.height = 1;
    cloud.width = pts.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    cloud.fields.clear();
    cloud.fields.push_back(makeField("x", 0));
    cloud.fields.push_back(makeField("y", 4));
    cloud.fields.push_back(makeField("z", 8));

    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    float* data = reinterpret_cast<float*>(cloud.data.data());
    for (size_t i = 0; i < pts.size(); ++i) {
        data[i*3 + 0] = pts[i].x();
        data[i*3 + 1] = pts[i].y();
        data[i*3 + 2] = pts[i].z();
    }
    return cloud;
}

std::vector<Eigen::Vector3f> circle(
    float radius,
    int num_points,Eigen::Vector3f offset)
{
    std::vector<Eigen::Vector3f> pts;
    for (int i = 0; i < num_points; ++i) {
        float theta = 2 * M_PI * i / num_points;
        pts.emplace_back(
            radius * cos(theta),
            radius * sin(theta),
            0.0f
        );
    }

    for (auto& p : pts) {
        p += offset;
    }
    return pts;
}

std::vector<Eigen::Vector3f> rectangle(
    float length,
    float width,
    int points_per_edge, Eigen::Vector3f offset)
{
    std::vector<Eigen::Vector3f> pts;

    auto edge = [&](Eigen::Vector3f a, Eigen::Vector3f b) {
        for (int i = 0; i < points_per_edge; ++i) {
            float t = float(i) / (points_per_edge - 1);
            pts.push_back(a + t * (b - a));
        }
    };

    Eigen::Vector3f A(-length/2, -width/2, 0);
    Eigen::Vector3f B( length/2, -width/2, 0);
    Eigen::Vector3f C( length/2,  width/2, 0);
    Eigen::Vector3f D(-length/2,  width/2, 0);

    edge(A, B);
    edge(B, C);
    edge(C, D);
    edge(D, A);

    std::vector<Eigen::Vector3f> res;
    for (auto& p : pts) {
        p += offset;
        res.push_back(p);
    }
    return res;
}

std::vector<Eigen::Vector3f> rectangleSolid(
    float length,
    float width,
    Eigen::Vector3f offset, float resolution = 0.05f)
{
    std::vector<Eigen::Vector3f> pts;
    for (float x = -length/2; x <= length/2; x += resolution) {
        for (float y = -width/2; y <= width/2; y += resolution) {
            pts.emplace_back(x, y, 0.0f);
        }
    }
    std::vector<Eigen::Vector3f> res;
    for (auto& p : pts) {
        p += offset;
        res.push_back(p);
    }
    return res;
}

std::vector<Eigen::Vector3f> transformShape(
    const std::vector<Eigen::Vector3f>& shape,
    const Eigen::Vector3f& pos,
    float yaw,Eigen::Vector3f offset)
{
    std::vector<Eigen::Vector3f> result;
    Eigen::Matrix3f R;
    R << cos(yaw), -sin(yaw), 0,
         sin(yaw),  cos(yaw), 0,
         0,         0,        1;

    for (auto& p : shape) {
        result.push_back(R * p + pos);
    }

    std::vector<Eigen::Vector3f> res;
    for (auto& p : result) {
        p += offset;
        res.push_back(p);
    }
    return res;
}

void pub_tf()
{

}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "publisher_data");
  ros::NodeHandle nh;

  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>("/odom", 50);

  ros::Publisher pub_dis =
      nh.advertise<std_msgs::Float64>("/dist_to_goal_th", 1);
  ros::Publisher pub_footprint_msg =
      nh.advertise<geometry_msgs::PolygonStamped>("/footprint", 1);
  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  ros::Publisher path_pub =
      nh.advertise<nav_msgs::Path>("/path", 1);
  ros::Publisher pub_target_velocity =
        nh.advertise<geometry_msgs::Twist>("/target_velocity", 1);
  ros::Publisher scan_pub_msg =
      nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
  ros::Publisher cloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/pub_pointcloud", 1);

  tf::TransformBroadcaster odom_broadcaster;

  float radius_, radius_x_, radius_y_, length_, width_, rectangle_x_, rectangle_y_;
  nh.param<float>("radius", radius_, 0.5);
  nh.param<float>("radius_x", radius_x_, 1.5);
  nh.param<float>("radius_y", radius_y_, -1.0);
  nh.param<float>("length", length_, 0.8);
  nh.param<float>("width", width_, 0.5);
  nh.param<float>("rectangle_x", rectangle_x_, 3.0);
  nh.param<float>("rectangle_y", rectangle_y_, 0.0);

  ros::Rate rate(20);
  std::vector<Eigen::Vector3f> points1 = circle(radius_,200,{radius_x_,radius_y_,0.0});
  std::vector<Eigen::Vector3f> points2 = rectangle(length_, width_, 50, {rectangle_x_, rectangle_y_, 0.0});
  std::vector<Eigen::Vector3f> points3 = rectangle(length_, width_, 50, {rectangle_x_, -rectangle_y_, 0.0});
  points1.insert(points1.end(), points2.begin(), points2.end());
  points1.insert(points1.end(), points3.begin(), points3.end());


  while (ros::ok())
  {
    sensor_msgs::PointCloud2 pointcloud2 = toPointCloud2(points1,"base_link");
    pub_odom(odom_pub, odom_broadcaster);
    pub_msg(pub_dis);
    pub_footprint(pub_footprint_msg);
    goal_pub_function(goal_pub);
    pub_path_function(path_pub);
    pub_scan(scan_pub_msg);
    cloud_pub_.publish(pointcloud2);

    rate.sleep();
  }

  return 0;
}