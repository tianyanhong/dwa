#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

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
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(q);
    odom_broadcaster.sendTransform(
        tf::StampedTransform(transform, current_time, "odom", "base_link"));
    tf::Transform transform2;
    transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0, 0, 0);
    transform2.setRotation(q);
    odom_broadcaster.sendTransform(
        tf::StampedTransform(transform2, current_time, "map", "odom"));

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
  goal.pose.position.y = 3.0;
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
  scan.range_min = 0.01;
  scan.range_max = 20.0;
  const int num = 361;
  scan.ranges.resize(num, scan.range_max);
  scan.intensities.resize(num, 0);

    // ✅ 正前方 5m 处障碍物
    int front_index = 185;
    for(int i= front_index;i < (front_index + 10);i++)
    {
        scan.ranges[i] = 2.0; 
    }

    int front_index_2 = 140;
    for(int i= front_index_2;i < (front_index_2 + 5);i++)
    {
        scan.ranges[i] = 1.5; 
    }
    scan.header.stamp = ros::Time::now();
    pub_scan_msg.publish(scan);

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
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate rate(20);

  while (ros::ok())
  {
    pub_odom(odom_pub, odom_broadcaster);
    pub_msg(pub_dis);
    pub_footprint(pub_footprint_msg);
    goal_pub_function(goal_pub);
    pub_path_function(path_pub);
    pub_scan(scan_pub_msg);

    rate.sleep();
  }

  return 0;
}