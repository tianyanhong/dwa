// Copyright 2020 amsl

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    : local_nh_("~"), odom_updated_(false), local_map_updated_(false), scan_updated_(false), has_reached_(false),
      use_speed_cost_(false), odom_not_subscribe_count_(0), local_map_not_subscribe_count_(0),
      scan_not_subscribe_count_(0), vehicle_(2.0, 0.8, 0.3)
{
  load_params();

  ROS_INFO("=== DWA Planner ===");
  print_params();

  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  candidate_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  selected_trajectory_pub_ = local_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  predict_footprints_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("predict_footprints", 1);
  finish_flag_pub_ = local_nh_.advertise<std_msgs::Bool>("finish_flag", 1);
  path_cloud_pub_ = local_nh_.advertise<sensor_msgs::PointCloud>("path_cloud", 10);
  pub_footprint_msg_ =
      local_nh_.advertise<geometry_msgs::PolygonStamped>("/footprint_2", 1);

  dist_to_goal_th_sub_ = nh_.subscribe("/dist_to_goal_th", 1, &DWAPlanner::dist_to_goal_th_callback, this);
  edge_on_global_path_sub_ = nh_.subscribe("/path", 1, &DWAPlanner::edge_on_global_path_callback, this);
  footprint_sub_ = nh_.subscribe("/footprint", 1, &DWAPlanner::footprint_callback, this);
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DWAPlanner::goal_callback, this);
  // local_map_sub_ = nh_.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);  不使用这个进行碰撞检测。
  odom_sub_ = nh_.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
  // target_velocity_sub_ = nh_.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);

  sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_); //仿真步长 1s/10 = 0.01s


  if (!use_footprint_)
  {
    footprint_ = vehicle_.makeFootprint();
  }


  if (!use_path_cost_)
    edge_points_on_path_ = nav_msgs::Path();
  if (!use_scan_as_input_)
    scan_updated_ = true;
  else
    local_map_updated_ = true;
}

DWAPlanner::State::State(void) : x_(0.0), y_(0.0), yaw_(0.0), velocity_(0.0), yawrate_(0.0) ,path_index_(0) {}

DWAPlanner::State::State(const double x, const double y, const double yaw, const double velocity, const double yawrate,const int path_index)
    : x_(x), y_(y), yaw_(yaw), velocity_(velocity), yawrate_(yawrate),path_index_(path_index)
{
}

DWAPlanner::Window::Window(void) : min_velocity_(0.0), max_velocity_(0.0), min_yawrate_(0.0), max_yawrate_(0.0) {}

void DWAPlanner::Window::show(void)
{
  ROS_INFO_STREAM("Window:");
  ROS_INFO_STREAM("\tVelocity:");
  ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
  ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
  ROS_INFO_STREAM("\tYawrate:");
  ROS_INFO_STREAM("\t\tmax: " << max_yawrate_);
  ROS_INFO_STREAM("\t\tmin: " << min_yawrate_);
}

DWAPlanner::Cost::Cost(void) : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0), total_cost_(0.0)
{
}

DWAPlanner::Cost::Cost(
    const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
    const float total_cost)
    : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
      total_cost_(total_cost)
{
}

void DWAPlanner::Cost::show(void)
{
  ROS_INFO_STREAM("Cost: " << total_cost_);
  ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
  ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
  ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
  ROS_INFO_STREAM("\tPath cost: " << path_cost_);
}

void DWAPlanner::Cost::calc_total_cost(void) { total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_; }

void DWAPlanner::goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  goal_msg_ = *msg;
  if (goal_msg_.value().header.frame_id != global_frame_)
  {
    try
    {
      listener_.transformPose(
          global_frame_, ros::Time(0), goal_msg_.value(), goal_msg_.value().header.frame_id, goal_msg_.value());
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void DWAPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
  if (use_scan_as_input_)
    create_obs_list(*msg);
  scan_not_subscribe_count_ = 0;
  scan_updated_ = true;
}

void DWAPlanner::local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  if (!use_scan_as_input_)
    create_obs_list(*msg);
  local_map_not_subscribe_count_ = 0;
  local_map_updated_ = true;
}

void DWAPlanner::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  current_cmd_vel_ = msg->twist.twist;
  odom_not_subscribe_count_ = 0;
  odom_updated_ = true;
}

void DWAPlanner::target_velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  target_velocity_ = std::min(msg->linear.x, max_velocity_);
  ROS_INFO_STREAM_THROTTLE(1.0, "target velocity was updated to " << target_velocity_ << " [m/s]");
}

void DWAPlanner::footprint_callback(const geometry_msgs::PolygonStampedPtr &msg)
{
  //对足迹点做一个外向的膨胀
  footprint_ = *msg; 
  for (auto &point : footprint_.value().polygon.points)
  {
    point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
    point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
  }
  //足迹点为长方体：将足迹点划分成6个区域
}

void DWAPlanner::dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg)
{
  dist_to_goal_th_ = msg->data;
  ROS_INFO_STREAM_THROTTLE(1.0, "distance to goal threshold was updated to " << dist_to_goal_th_ << " [m]");
}

void DWAPlanner::edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg)
{
  if (!use_path_cost_)
    return;
  edge_points_on_path_ = *msg;
  try
  {
    int n = 0;
    for (auto &pose : edge_points_on_path_.value().poses)
    {
      listener_.transformPose(robot_frame_, ros::Time(0), pose, msg->header.frame_id, pose);
      //将路径转为states;
      State state;
      state.x_ = pose.pose.position.x;
      state.y_ = pose.pose.position.y;
      state.yaw_ = tf::getYaw(pose.pose.orientation);
      state.path_index_ = n++;
      local_path_.push_back(state);
    }
      
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

std::vector<DWAPlanner::State>
DWAPlanner::dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories)
{
  Cost min_cost(0.0, 0.0, 0.0, 0.0, 1.0);//1e6
  const Window dynamic_window = calc_dynamic_window();

  std::vector<State> best_traj;
  best_traj.resize(sim_time_samples_);
  std::vector<Cost> costs;
  const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1); //速度3和角速度20的采样
  costs.reserve(costs_size);

  //速度和角速度的划分

  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double yawrate_resolution =
      std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);
  

  int available_traj_count = 0; //有效的轨迹计数
  for (int i = 0; i < velocity_samples_; i++)
  {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++)
    {
      std::pair<std::vector<State>, bool> traj;
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;
      if (v < slow_velocity_th_) //0.1
        y = y > 0 ? std::max(y, min_yawrate_) : std::min(y, -min_yawrate_);
      //1.计算轨迹
      traj.first = generate_trajectory(v, y); 
      //放置在有效路径的末尾

      // //转到有效轨迹的末端
      // //2.评估
      // const Cost cost = evaluate_trajectory(traj.first, goal);
      // costs.push_back(cost);
      // if (cost.obs_cost_ == 1e6)
      // {
      //   traj.second = false;
      // }
      // else
      // {
      //   traj.second = true;
      //   available_traj_count++;
      // }
      trajectories.push_back(traj);
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_)
    {
      std::pair<std::vector<State>, bool> traj;
      //1.计算轨迹
      traj.first = generate_trajectory(v, 0.0);
      //2.评估
      // const Cost cost = evaluate_trajectory(traj.first, goal);
      // costs.push_back(cost);
      // if (cost.obs_cost_ == 1e6 || calc_path_length(trajectories[i].first) < 1.5 )
      // {
      //   traj.second = false;
      // }
      // else
      // {
      //   traj.second = true;
      //   available_traj_count++;
      // }
      trajectories.push_back(traj);
    }
  }



  // if (available_traj_count == 0)
  // {
  //   ROS_ERROR_THROTTLE(1.0, "No available trajectory");
  //   best_traj = generate_trajectory(0.0, 0.0);
  // }
  // else
  // {
  //   //最优轨迹选择逻辑
  //   normalize_costs(costs);
  //   for (int i = 0; i < costs.size(); i++)
  //   {
  //     // ROS_INFO("=== 0 %lf",costs[i].obs_cost_);
  //     if (costs[i].obs_cost_ != 1e6)
  //     {
  //       costs[i].to_goal_cost_ *= to_goal_cost_gain_; //0.3
  //       costs[i].obs_cost_ *= obs_cost_gain_; //1.0
  //       costs[i].speed_cost_ *= speed_cost_gain_; //speed_cost_gain_ = 0.4
  //       costs[i].path_cost_ *= path_cost_gain_;  //path_cost_gain_ = 0.4 
  //       costs[i].calc_total_cost();
  //       // ROS_INFO("=== 1 %lf",costs[i].total_cost_);
  //       // calc_path_length(trajectories[i].first);
  //       if(calc_path_length(trajectories[i].first) > 1.5 )
  //       {
  //         best_traj = trajectories[i].first; 
  //       }

  //       if (costs[i].total_cost_ < min_cost.total_cost_) //最小的总损失  是怎么定义的,存在四个~~
  //       {
  //         // ROS_INFO("costs[i].total_cost_ %lf",costs[i].total_cost_); 
  //         min_cost = costs[i];
  //         if(calc_path_length(trajectories[i].first) > 2.7 )
  //         {
  //           best_traj = trajectories[i].first; 
  //         }
          
  //       }
  //     }
  //   }
  // }

  // ROS_INFO("===");//为什么是（0，0）
  // ROS_INFO_STREAM("(v, y) = (" << best_traj.front().velocity_ << ", " << best_traj.front().yawrate_ << ")");
  // min_cost.show(); //展示损失~~~~
  // ROS_INFO_STREAM("num of trajectories available: " << available_traj_count << " of " << trajectories.size());
  // ROS_INFO(" ");
  // ROS_INFO("%ld",best_traj.size());

  return best_traj;
}

void DWAPlanner::dwa_planning_dynamic(const DWAPlanner::State& current, const Eigen::Vector3d &goal, int depth,std::pair<std::vector<State>, bool> &traj)
{
  //终止的条件:达到了采样的站点  或者提前到达目标点
  if(traj.first.size() >= sim_time_samples_ )
  {
    trajectories_results_.push_back(traj);
    // if(calc_path_length(traj.first) > 0 && calc_to_goal_cost(traj.first, goal) < 1.0)
    // {
    //   trajectories_results_.push_back(traj);
    // }
    return;
  }
  
  std::vector<Cost> costs;
  const Window dynamic_window = calc_dynamic_window_dynamic(current);
  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double yawrate_resolution =
      std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);

  int available_traj_count = 0; //有效的轨迹计数
  for (int i = 0; i < velocity_samples_; i++)
  {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++)
    {
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;

      State next = current;
      motion(next,v,y);
      traj.first.push_back(next);
      dwa_planning_dynamic(next, goal, depth + 1, traj);
      traj.second = true;
      traj.first.pop_back();
    }
  }
}

void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> &costs)
{
  Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

  for (const auto &cost : costs)
  {
    if (cost.obs_cost_ != 1e6)
    {
      min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
      max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
      min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
      max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
      if (use_speed_cost_)
      {
        min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
        max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
      }
      if (use_path_cost_)
      {
        min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
        max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
      }
    }
  }

  for (auto &cost : costs)
  {
    if (cost.obs_cost_ != 1e6)
    {
      cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_ + DBL_EPSILON);
      cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
                           (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + DBL_EPSILON);
      if (use_speed_cost_)
        cost.speed_cost_ =
            (cost.speed_cost_ - min_cost.speed_cost_) / (max_cost.speed_cost_ - min_cost.speed_cost_ + DBL_EPSILON);
      if (use_path_cost_)
        cost.path_cost_ =
            (cost.path_cost_ - min_cost.path_cost_) / (max_cost.path_cost_ - min_cost.path_cost_ + DBL_EPSILON);
    }
  }
}

void DWAPlanner::process(void)
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {

    geometry_msgs::Twist cmd_vel;

    if (can_move())
      cmd_vel = calc_cmd_vel();
    ROS_INFO("can move");
    velocity_pub_.publish(cmd_vel);
    finish_flag_pub_.publish(has_finished_);
    if (has_finished_.data)
      ros::Duration(sleep_time_after_finish_).sleep();

    if (use_scan_as_input_)
      scan_updated_ = false;
    else
      local_map_updated_ = false;
    odom_updated_ = false;
    has_finished_.data = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool DWAPlanner::can_move(void)
{
  if (!footprint_.has_value())
    ROS_WARN_THROTTLE(1.0, "Robot Footprint has not been updated");
  if (!goal_msg_.has_value())
    ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
  if (!edge_points_on_path_.has_value())
    ROS_WARN_THROTTLE(1.0, "Edge on global path has not been updated");
  if (subscribe_count_th_ < odom_not_subscribe_count_)
    ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
  // if (subscribe_count_th_ < local_map_not_subscribe_count_)
  //   ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
  if (subscribe_count_th_ < scan_not_subscribe_count_)
    ROS_WARN_THROTTLE(1.0, "Scan has not been updated");

  if (!odom_updated_)
    odom_not_subscribe_count_++;
  if (!local_map_updated_)
    local_map_not_subscribe_count_++;
  if (!scan_updated_)
    scan_not_subscribe_count_++;

  if (footprint_.has_value() && goal_msg_.has_value() && edge_points_on_path_.has_value() &&
      odom_not_subscribe_count_ <= subscribe_count_th_ &&
      scan_not_subscribe_count_ <= subscribe_count_th_) //&& local_map_not_subscribe_count_ <= subscribe_count_th_ 
    return true;
  else
    return false;
}

geometry_msgs::Twist DWAPlanner::calc_cmd_vel(void)
{
  // CollisionPointsByRegion result = classifyCollidingPointsByRegion(local_path_,vehicle_,obstacles_points_);

  geometry_msgs::Twist cmd_vel;
  std::pair<std::vector<State>, bool> best_traj;
  std::vector<std::pair<std::vector<State>, bool>> trajectories;
  const size_t trajectories_size = velocity_samples_ * (yawrate_samples_ + 1); //速度和角速度的采样
  trajectories.reserve(trajectories_size);

  geometry_msgs::PoseStamped goal_;
  try
  {
    listener_.transformPose(robot_frame_, ros::Time(0), goal_msg_.value(), goal_msg_.value().header.frame_id, goal_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  const Eigen::Vector3d goal(goal_.pose.position.x, goal_.pose.position.y, tf::getYaw(goal_.pose.orientation));

  //目标在机器人正前方45度之外
  const double angle_to_goal = atan2(goal.y(), goal.x()); 
  if (M_PI / 4.0 < fabs(angle_to_goal))
    use_speed_cost_ = true;
  State current(0.0,0.0,0.0,0.0,0.0,0);
  State end(goal_.pose.position.x,goal_.pose.position.y,0.0,0.0,0.0,0);
  
  //机器人当前位置到目标的平面的距离
  if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_)
  {
    //订阅存在障碍物的路径在障碍物的路径上进行搜索
    //障碍物的偏离量怎么计算
    std::pair<std::vector<State>, bool> traj;
    // dwa_planning_dynamic(current,goal,0,traj);

    std::vector<State> path = AstartSearch2(current,goal);
        ROS_INFO("can move 2");

    for(auto p:path)
    {
      best_traj.first.push_back(p);
    }

    // if (can_adjust_robot_direction(goal)) //有存在碰撞的路径
    // {
    //   cmd_vel.angular.z = angle_to_goal > 0 ? std::min(angle_to_goal, max_in_place_yawrate_)
    //                                         : std::max(angle_to_goal, -max_in_place_yawrate_);
    //   cmd_vel.angular.z = cmd_vel.angular.z > 0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_)
    //                                             : std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
    //   best_traj.first = generate_trajectory(cmd_vel.angular.z, goal); //cmd_vel控制的，目标点
    //   ROS_INFO("best_traj.first = %ld",best_traj.first.size()); 
    //   publishPathAsPointCloud(path_cloud_pub_,best_traj.first);
    //   trajectories.push_back(best_traj);
    // }
    // else
    // {
    //    //不能调整就是执行这个
    //   // best_traj.first = dwa_planning(goal, trajectories);
    //   //保存有效路径的终点作为起点再次进行dwa_planner.

    //   cmd_vel.linear.x = best_traj.first.front().velocity_;
    //   cmd_vel.angular.z = best_traj.first.front().yawrate_;
    // }
  }
  else
  {
    has_reached_ = true;
    if (turn_direction_th_ < fabs(goal[2]))
    {
      cmd_vel.angular.z =
          goal[2] > 0 ? std::min(goal[2], max_in_place_yawrate_) : std::max(goal[2], -max_in_place_yawrate_);
      cmd_vel.angular.z = cmd_vel.angular.z > 0 ? std::max(cmd_vel.angular.z, min_in_place_yawrate_)
                                                : std::min(cmd_vel.angular.z, -min_in_place_yawrate_);
    }
    else
    {
      has_finished_.data = true;
      has_reached_ = false;
    }
    best_traj.first = generate_trajectory(cmd_vel.linear.x, cmd_vel.angular.z);
    trajectories.push_back(best_traj);
  }


  // ROS_INFO("trajectories_size = %ld",trajectories_size);
  // for (int i = 0; i < trajectories_size; i++)
  //   trajectories.push_back(trajectories.front());
  ROS_INFO("trajectories size = %ld",best_traj_.first.size()); 

  visualize_trajectory(best_traj_.first, selected_trajectory_pub_);


  visualize_trajectories(trajectories_res_, candidate_trajectories_pub_);  

  // ROS_INFO("trajectories = %ld",trajectories.size()); 
  visualize_footprints(best_traj_.first, predict_footprints_pub_);
  trajectories_results_.clear();
  trajectories_res_.clear();

  use_speed_cost_ = false;
  // std::this_thread::sleep_for(std::chrono::minutes(1));

  return cmd_vel;
}

bool DWAPlanner::can_adjust_robot_direction(const Eigen::Vector3d &goal)
{

  const double angle_to_goal = atan2(goal.y(), goal.x());
  //ROS_INFO("if adjust robot direction %lf",fabs(angle_to_goal));  //0.005
  if (fabs(angle_to_goal) < angle_to_goal_th_)
  {
      //ROS_INFO("if adjust robot direction 0");
      return false;
  }

  // ROS_INFO("if adjust robot direction 1");
  const double yawrate = std::min(std::max(angle_to_goal, -max_in_place_yawrate_), max_in_place_yawrate_);
  std::vector<State> traj = generate_trajectory(yawrate, goal); //给一个角速度和目标点
  // ROS_INFO("if adjust robot direction 2");
  //生成轨迹 检测是否碰撞
  // ROS_INFO("check_collision %d",check_collision(traj)); //false
  if (!check_collision(traj)) // 
  {
    ROS_INFO("if adjust robot direction 3 ");
    return true;
  }
  else
  {
    ROS_INFO("if adjust robot direction 4 ");
    return false;
  }
}
    

bool DWAPlanner::check_collision(const std::vector<State> &traj)
{
  if (!use_footprint_)
    return false;
  //每个路径点的足迹进行障碍物的检测
  // ROS_INFO("traj size %ld",traj.size());// 10
  // ROS_INFO("obs_list_ size %ld",obs_list_.poses.size());//91
  for (const auto &state : traj)
  {

    for (const auto &obs : obs_list_.poses)
    {

      const geometry_msgs::PolygonStamped footprint = move_footprint(state);
      if (is_inside_of_robot(obs.position, footprint,state))
        return true;
    }
  }


  return false;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
  Window window;
  // ROS_INFO("current_cmd_vel_ x = %lf, target_velocity_ = %lf", current_cmd_vel_.linear.x,target_velocity_);
  window.min_velocity_ = std::max((current_cmd_vel_.linear.x - max_deceleration_ * sim_period_), min_velocity_);
  window.max_velocity_ = std::min((current_cmd_vel_.linear.x + max_acceleration_ * sim_period_), target_velocity_);
  window.min_yawrate_ = std::max((current_cmd_vel_.angular.z - max_d_yawrate_ * sim_period_), -max_yawrate_);
  window.max_yawrate_ = std::min((current_cmd_vel_.angular.z + max_d_yawrate_ * sim_period_), max_yawrate_);
  return window;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window_dynamic(const State current)
{
  Window window;
  window.min_velocity_ = std::max((current.velocity_ - 0.3), min_velocity_); //max_deceleration_  2.0  
  window.max_velocity_ = std::min((current.velocity_ + 0.3), target_velocity_);//target_velocity_
  window.min_yawrate_ = std::max((current.yawrate_ - 0.3), -max_yawrate_);//max_d_yawrate_ 0.5
  window.max_yawrate_ = std::min((current.yawrate_ + 0.3), max_yawrate_); //角速度的灯饰
  // if(current.velocity_ !=0)
  // {
  //   ROS_INFO("min_velocity_  = %lf,max_velocity_ = %lf, max_yawrate_ = %lf ", min_velocity_ ,max_velocity_,max_yawrate_);
  //   ROS_INFO("current.velocity_  = %lf,current.yawrate_ = %lf", current.velocity_ ,current.yawrate_);
  //   ROS_INFO("window.min_velocity_ = %lf,window.max_velocity_ = %lf", window.min_velocity_ , window.max_velocity_ );
  //   ROS_INFO("window.min_yawrate_ = %lf,window.min_yawrate_ = %lf", window.min_yawrate_ , window.max_yawrate_ );
  // }

  return window;
}
/*
  计算轨迹终点到目标点之间的欧几里得距离，作为到大目标的代价。
*/
float DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
  Eigen::Vector3d last_position(traj.back().x_, traj.back().y_, traj.back().yaw_);
  return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_obs_cost(const std::vector<State> &traj)
{
  float min_dist = obs_range_;
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list_.poses) //障碍物的点到足迹的最小距离。
    {
      float dist;
      if (use_footprint_)
        dist = calc_dist_from_robot(obs.position, state);
      else
        dist = hypot((state.x_ - obs.position.x), (state.y_ - obs.position.y)) - robot_radius_ - footprint_padding_; //可通行距离

      if (dist < DBL_EPSILON)
        return 1e6;
      min_dist = std::min(min_dist, dist);
    }
  }
  return obs_range_ - min_dist; //为正为负  接近0最好
}

bool DWAPlanner::calc_obs_cost3(const std::vector<State> &traj)
{
  for (const auto &state : traj)
  {
    for (const auto &obs : obs_list_.poses) //障碍物的点到足迹的最小距离。
    {
      float dist;
      if (use_footprint_)
      {
        const geometry_msgs::PolygonStamped footprint = move_footprint(state);
        if(is_inside_of_robot(obs.position, footprint, state))
        {
          return false;
        }
      }
    }
  }
  return true; //为正为负  接近0最好
}


float DWAPlanner::calc_speed_cost(const std::vector<State> &traj)
{
  if (!use_speed_cost_)
    return 0.0;
  const Window dynamic_window = calc_dynamic_window();
  return dynamic_window.max_velocity_ - traj.front().velocity_;
}
//最后一个点到路径的距离。
float DWAPlanner::calc_path_cost(const std::vector<State> &traj)
{
  float dis = 0.0;
  if (!use_path_cost_)
    return 0.0;
  else
  {
    // for(const auto& state:traj)
    // {
      dis += calc_dist_to_path(traj.back());
    // }
  }
  return dis;
}

float DWAPlanner::calc_predict_path_cost(const std::vector<DWAPlanner::State> &traj,const DWAPlanner::State& goal)
{
  double dis = FLT_MAX;
  if (!use_path_cost_)
    return 0.0;
  else
  {
    for(const auto& state:traj)
    {
      dis = std::min(heuristic(state,goal),dis);
    }
  }
  return dis;
}

float DWAPlanner::calc_dist_to_path(const State state) //状态量到路径的垂直距离
{
  geometry_msgs::Point edge_point1 = edge_points_on_path_.value().poses.front().pose.position;
  geometry_msgs::Point edge_point2 = edge_points_on_path_.value().poses.back().pose.position;
  const float a = edge_point2.y - edge_point1.y;
  const float b = -(edge_point2.x - edge_point1.x);
  const float c = -a * edge_point1.x - b * edge_point1.y;

  return fabs(a * state.x_ + b * state.y_ + c) / (hypot(a, b) + DBL_EPSILON);
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double velocity, const double yawrate)
{
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = 0; i < sim_time_samples_; i++)
  {
    motion(state, velocity, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

//路径的末尾点，转到第i个路径的末尾点。
// std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double velocity, const double yawrate)
// {
//   std::vector<State> trajectory;
//   trajectory.resize(sim_time_samples_);
//   State state;
//   for (int i = 0; i < sim_time_samples_; i++)
//   {
//     motion(state, velocity, yawrate);
//     trajectory[i] = state;
//   }
//   return trajectory;
// }


std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory_with_depth(const double velocity, const double yawrate,const Eigen::Vector3d &goal, int depth)
{
  ROS_INFO("generate_trajectory_with_depth %d",depth);

  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  State state;
  for (int i = depth; i < sim_time_samples_; i++)
  {
    motion(state, velocity, yawrate);//深度加1
    
    //新的状态量下，进行新的dwa搜索
    // dwa_planning_dynamic(state,goal,++depth,);

    trajectory[depth] = state;
  }
  return trajectory;
}

std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory(const double yawrate, const Eigen::Vector3d &goal)
{
  const double target_direction = atan2(goal.y(), goal.x()) > 0 ? sim_direction_ : -sim_direction_; 
  const double predict_time = target_direction / (yawrate + DBL_EPSILON); 
  ROS_INFO("target_direction  = %lf",target_direction);//1.57
  ROS_INFO("predict_time  = %lf",predict_time);//313
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_); //10
  State state;
  for (int i = 0; i < sim_time_samples_; i++)
  {
    motion(state, 1.0, yawrate); //由状态量为0.0，速度为1.0，角速度为 yawrate = std::min(std::max(angle_to_goal, -max_in_place_yawrate_), max_in_place_yawrate_);//max_in_place_yawrate_ = 0.6
    trajectory[i] = state;
  }
  return trajectory;
}

DWAPlanner::Cost DWAPlanner::evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal)
{
  Cost cost;
  // cost.to_goal_cost_ = calc_to_goal_cost(trajectory, goal);
  cost.obs_cost_ = calc_obs_cost(trajectory);
  cost.speed_cost_ = calc_speed_cost(trajectory);
  cost.path_cost_ = calc_path_cost(trajectory);
  cost.calc_total_cost();
  return cost;
}

geometry_msgs::Point DWAPlanner::calc_intersection(
    const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint)
{
  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    const Eigen::Vector3d vector_A(obstacle.x, obstacle.y, 0.0);
    const Eigen::Vector3d vector_B(state.x_, state.y_, 0.0);
    const Eigen::Vector3d vector_C(footprint.polygon.points[i].x, footprint.polygon.points[i].y, 0.0);
    Eigen::Vector3d vector_D(0.0, 0.0, 0.0);
    if (i != footprint.polygon.points.size() - 1)
      vector_D << footprint.polygon.points[i + 1].x, footprint.polygon.points[i + 1].y, 0.0;
    else
      vector_D << footprint.polygon.points[0].x, footprint.polygon.points[0].y, 0.0;

    const double deno = (vector_B - vector_A).cross(vector_D - vector_C).z();
    const double s = (vector_C - vector_A).cross(vector_D - vector_C).z() / deno;
    const double t = (vector_B - vector_A).cross(vector_A - vector_C).z() / deno;

    geometry_msgs::Point point;
    point.x = vector_A.x() + s * (vector_B - vector_A).x();
    point.y = vector_A.y() + s * (vector_B - vector_A).y();

    // cross
    if (!(s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t))
      return point;
  }

  geometry_msgs::Point point;
  point.x = 1e6;
  point.y = 1e6;
  return point;
}

float DWAPlanner::calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state)
{
  const geometry_msgs::PolygonStamped footprint = move_footprint(state);
  if (is_inside_of_robot(obstacle, footprint,state))
  {
    return 0.0;
  }
  else
  {
    geometry_msgs::Point intersection = calc_intersection(obstacle, state, footprint);
    return hypot((obstacle.x - intersection.x), (obstacle.y - intersection.y));
  }
}

//给定机器人的目标位姿（x,y,yaw）,返回该位姿下的机器人轮廓。
/*
碰撞检测，costmap可视化，轨迹评分
*/
geometry_msgs::PolygonStamped DWAPlanner::move_footprint(const State &target_pose)
{
  geometry_msgs::PolygonStamped footprint;
  if (use_footprint_)
  {
    footprint = footprint_.value();
  }
  else
  {
    const int plot_num = 20;
    for (int i = 0; i < plot_num; i++)
    {
      geometry_msgs::Point32 point;
      point.x = (robot_radius_ + footprint_padding_) * cos(2 * M_PI * i / plot_num);
      point.y = robot_radius_ * sin(2 * M_PI * i / plot_num);
      footprint.polygon.points.push_back(point);
    }
  }

  footprint.header.stamp = ros::Time::now();

  for (auto &point : footprint.polygon.points)
  {
    Eigen::VectorXf point_in(2);
    point_in << point.x, point.y;
    Eigen::Matrix2f rot;
    rot = Eigen::Rotation2Df(target_pose.yaw_);
    const Eigen::VectorXf point_out = rot * point_in;

    point.x = point_out.x() + target_pose.x_;
    point.y = point_out.y() + target_pose.y_;
  }
  pub_footprint_msg_.publish(footprint);
  return footprint;
}

bool DWAPlanner::isInsidePolygon(const geometry_msgs::Point& p,
                     const geometry_msgs::PolygonStamped& poly)
{
    int crossings = 0;
    const auto& pts = poly.polygon.points;

    for (size_t i = 0; i < pts.size(); ++i) {
        size_t j = (i + 1) % pts.size();

        if (((pts[i].y > p.y) != (pts[j].y > p.y)) &&
            (p.x < (pts[j].x - pts[i].x) * (p.y - pts[i].y) /
                           (pts[j].y - pts[i].y) + pts[i].x))
            crossings++;
    }
    return (crossings % 2) == 1;
}

bool DWAPlanner::is_inside_of_robot(
    const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped footprint, const State &state)
{
  geometry_msgs::Point32 state_point;
  state_point.x = state.x_;
  state_point.y = state.y_;

  for (int i = 0; i < footprint.polygon.points.size(); i++)
  {
    geometry_msgs::Polygon triangle;
    triangle.points.push_back(state_point);
    triangle.points.push_back(footprint.polygon.points[i]);

    if (i != footprint.polygon.points.size() - 1)
      triangle.points.push_back(footprint.polygon.points[i + 1]);
    else
      triangle.points.push_back(footprint.polygon.points[0]);

    if (is_inside_of_triangle(obstacle, triangle))
      return true;
  }

  return false;
}

bool DWAPlanner::is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle)
{
  if (triangle.points.size() != 3)
  {
    ROS_ERROR("Not triangle");
    exit(1);
  }

  const Eigen::Vector3d vector_A(triangle.points[0].x, triangle.points[0].y, 0.0);
  const Eigen::Vector3d vector_B(triangle.points[1].x, triangle.points[1].y, 0.0);
  const Eigen::Vector3d vector_C(triangle.points[2].x, triangle.points[2].y, 0.0);
  const Eigen::Vector3d vector_P(target_point.x, target_point.y, 0.0);

  const Eigen::Vector3d vector_AB = vector_B - vector_A;
  const Eigen::Vector3d vector_BP = vector_P - vector_B;
  const Eigen::Vector3d cross1 = vector_AB.cross(vector_BP);

  const Eigen::Vector3d vector_BC = vector_C - vector_B;
  const Eigen::Vector3d vector_CP = vector_P - vector_C;
  const Eigen::Vector3d cross2 = vector_BC.cross(vector_CP);

  const Eigen::Vector3d vector_CA = vector_A - vector_C;
  const Eigen::Vector3d vector_AP = vector_P - vector_A;
  const Eigen::Vector3d cross3 = vector_CA.cross(vector_AP);

  if ((0 < cross1.z() && 0 < cross2.z() && 0 < cross3.z()) || (cross1.z() < 0 && cross2.z() < 0 && cross3.z() < 0))
    return true;
  else
    return false;
}

void DWAPlanner::motion(State &state, const double velocity, const double yawrate)
{
  
  state.yaw_ += yawrate * sim_time_step; //
  state.x_ += velocity * std::cos(state.yaw_) * sim_time_step;
  state.y_ += velocity * std::sin(state.yaw_) * sim_time_step;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
  state.path_index_++;
  
}

void DWAPlanner::create_obs_list(const sensor_msgs::LaserScan &scan)
{
  obs_list_.poses.clear();
  obstacles_points_.clear();
  float angle = scan.angle_min;
  const int angle_index_step = static_cast<int>(angle_resolution_ / scan.angle_increment);
  for (int i = 0; i < scan.ranges.size(); i++)
  {
    const float r = scan.ranges[i];
    if (r < scan.range_min || scan.range_max < r || i % angle_index_step != 0)
    {
      angle += scan.angle_increment;
      continue;
    }
    geometry_msgs::Pose pose;
   
    pose.position.x = r * cos(angle);
    pose.position.y = r * sin(angle);
    obs_list_.poses.push_back(pose);
    geometry_msgs::Point point = pose.position;
    obstacles_points_.push_back(point);
    angle += scan.angle_increment;
  }
}

void DWAPlanner::create_obs_list(const nav_msgs::OccupancyGrid &map)
{
  obs_list_.poses.clear();
  const double max_search_dist = hypot(map.info.origin.position.x, map.info.origin.position.y);
  for (float angle = -M_PI; angle <= M_PI; angle += angle_resolution_)
  {
    for (float dist = 0.0; dist <= max_search_dist; dist += map.info.resolution)
    {
      geometry_msgs::Pose pose;
      pose.position.x = dist * cos(angle);
      pose.position.y = dist * sin(angle);
      const int index_x = floor((pose.position.x - map.info.origin.position.x) / map.info.resolution);
      const int index_y = floor((pose.position.y - map.info.origin.position.y) / map.info.resolution);

      if ((0 <= index_x && index_x < map.info.width) && (0 <= index_y && index_y < map.info.height))
      {
        if (map.data[index_x + index_y * map.info.width] == 100)
        {
          obs_list_.poses.push_back(pose);
          break;
        }
      }
    }
  }
}

visualization_msgs::Marker DWAPlanner::create_marker_msg(
    const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
    const geometry_msgs::PolygonStamped &footprint)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = robot_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.scale.x = scale;
  marker.color = color;
  marker.color.a = 0.8;
  marker.lifetime = ros::Duration(1 / hz_);

  geometry_msgs::Point p;
  if (footprint.polygon.points.empty())
  {
    for (const auto &point : trajectory)
    {
      p.x = point.x_;
      p.y = point.y_;
      
      marker.points.push_back(p);
    }
  }
  else
  {
    for (const auto &point : footprint.polygon.points)
    {
      p.x = point.x;
      p.y = point.y;
      marker.points.push_back(p);
    }
    p.x = footprint.polygon.points.front().x;
    p.y = footprint.polygon.points.front().y;
    marker.points.push_back(p);
  }

  return marker;
}

void DWAPlanner::visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub)
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  visualization_msgs::Marker v_trajectory = create_marker_msg(0, v_path_width_, color, trajectory);
  // v_trajectory.lifetime = ros::Duration(60.0); 
  pub.publish(v_trajectory);
}

void DWAPlanner::visualize_trajectories(
    const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  
  for (int i = 0; i < trajectories.size(); i++)
  {
    std_msgs::ColorRGBA color;
    if (trajectories[i].second)
    {
      color.g = 1.0;
    }
    else
    {
      color.r = 0.5;
      color.b = 0.5;
    }
    visualization_msgs::Marker v_trajectory = create_marker_msg(i, v_path_width_ * 0.4, color, trajectories[i].first);
    v_trajectory.lifetime = ros::Duration(60.0); 
    v_trajectories.markers.push_back(v_trajectory);
  }
  // ROS_INFO("pub traj");
  pub.publish(v_trajectories);
}

void DWAPlanner::visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub)
{
  std_msgs::ColorRGBA color;
  color.b = 1.0;
  visualization_msgs::MarkerArray v_footprints;
  for (int i = 0; i < trajectory.size(); i++)
  {
    const geometry_msgs::PolygonStamped footprint = move_footprint(trajectory[i]);
    visualization_msgs::Marker v_footprint = create_marker_msg(i, v_path_width_ * 0.2, color, trajectory, footprint);
    v_footprint.lifetime = ros::Duration(60.0); 
    v_footprints.markers.push_back(v_footprint);
  }
  pub.publish(v_footprints);
}


void DWAPlanner::publishPathAsPointCloud(
    ros::Publisher& pub,
    const std::vector<State>& states)
{
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";  // 根据你的坐标系修改

    cloud.points.reserve(states.size());

    // 可选：添加额外通道
    cloud.channels.resize(3);
    cloud.channels[0].name = "yaw";
    cloud.channels[1].name = "velocity";
    cloud.channels[2].name = "yawrate";

    for (const auto& s : states)
    {
        geometry_msgs::Point32 p;
        p.x = s.x_;
        p.y = s.y_;
        p.z = 0.0;  // 2D 路径
        cloud.points.push_back(p);

        cloud.channels[0].values.push_back(s.yaw_);
        cloud.channels[1].values.push_back(s.velocity_);
        cloud.channels[2].values.push_back(s.yawrate_);
    }

    pub.publish(cloud);
}


double DWAPlanner::calc_path_length(const std::vector<State>& traj)
{
  if (traj.size() < 2)
    return 0.0;

  double length = 0.0;
  for (size_t i = 1; i < traj.size(); ++i)
  {
    double dx = traj[i].x_ - traj[i - 1].x_;
    double dy = traj[i].y_ - traj[i - 1].y_;
    length += std::sqrt(dx * dx + dy * dy);
  }
  // ROS_INFO("length = %lf",length);
  return length;
}

void DWAPlanner::searchPath(const State& current, const Eigen::Vector3d goal, int depth)
{
  //1.终止条件
  if(depth > 10 || dist_to_goal_th_ < goal.segment(0, 2).norm())
  {
    return;
  }
  //2.当前层dwa采样，新的滑动窗口、速度和角速度的分辨率、生成的新的轨迹和新的起始搜索点
}


DWAPlanner::FootprintRegion DWAPlanner::getRegion(
    const geometry_msgs::Point& point,
    const VehicleParams& robot)
{

  const bool is_left  = point.y >  0.0;
  const bool is_right = point.y <= 0.0;

  // 纵向分段（上中下）
  const double front_mid_boundary =  robot.length * 0.6 - robot.rear_offset;
  const double rear_mid_boundary  = robot.length * 0.3 - robot.rear_offset;

  const bool is_forward  = point.x >  front_mid_boundary;
  const bool is_backward = point.x <  rear_mid_boundary;
  const bool is_middle   = !is_forward && !is_backward;

  if (is_left)
  {
    if (is_forward)  return FootprintRegion::LEFT_FRONT;
    if (is_middle)   return FootprintRegion::LEFT_MIDDLE;
    if (is_backward) return FootprintRegion::LEFT_REAR;
  }
  else if (is_right)
  {
    if (is_forward)  return FootprintRegion::RIGHT_FRONT;
    if (is_middle)   return FootprintRegion::RIGHT_MIDDLE;
    if (is_backward) return FootprintRegion::RIGHT_REAR;
  }

  return FootprintRegion::UNKNOWN;
}

bool DWAPlanner::isPathPointInCollision(
    const State& state,
    const std::vector<geometry_msgs::Point>& obstacles)
{
  for (const auto& obs : obstacles)
  {
      const geometry_msgs::PolygonStamped footprint = move_footprint(state);
    if (is_inside_of_robot(obs,footprint,state))
    {
      return true;
    }
  }
  return false;
}

std::vector<DWAPlanner::State> DWAPlanner::selectCollidingPathPoints(
    const std::vector<DWAPlanner::State>& path,
    const std::vector<geometry_msgs::Point>& obstacles)
{
  std::vector<State> colliding_points;
  for (const auto& state : path)
  {
    if (isPathPointInCollision(state, obstacles))
    {
      colliding_points.push_back(state);
    }
  }
  return colliding_points;
}

DWAPlanner::CollisionPointsByRegion DWAPlanner::classifyCollidingPointsByRegion(
   std::vector<DWAPlanner::State>& path,
    const VehicleParams& vehicle,
    const std::vector<geometry_msgs::Point>& obstacles)
{
  CollisionPointsByRegion result;
  bool if_same_side = true;
  std::optional<bool> collision_on_left; // true=左, false=右
      ROS_INFO("path %ld",path.size());
  for (const auto& state : path)
  {
              
    // 2️⃣ 判断是否与任意障碍物碰撞
    for (const auto& obs : obstacles_points_)
    {
      
      const geometry_msgs::PolygonStamped footprint = move_footprint(state);

      if (isInsidePolygon(obs,footprint))
      {
        // 3️⃣ 判断障碍物属于哪个区域
        FootprintRegion region = getRegion(obs,vehicle_);
        // 4️⃣ 归入对应区域
        result[region].push_back(state);
        // 5 障碍物是否在同一侧
        bool is_left_region = (region == FootprintRegion::LEFT_FRONT || region == FootprintRegion::LEFT_MIDDLE || region == FootprintRegion::LEFT_REAR);
        // 第一次碰撞：记录侧别
        if (!collision_on_left.has_value())
        {
          collision_on_left = is_left_region;
        }
        else
        {
          // 后续碰撞：必须和第一次同侧
          if (*collision_on_left != is_left_region)
          {
            if_same_side = false;
          }
        }
        break; // 该路径点已判定为碰撞
      }
    }
  }

  if(if_same_side) //平移中间1/3的路径点
  {
    ROS_INFO("if_same_side %d",result.size());
    for (auto& [region, states] : result)
    {
        ROS_INFO("if_same_side %s",region);

      if(region == FootprintRegion::LEFT_MIDDLE)
      {
        ROS_INFO("LEFT_MIDDLE ");
        for (auto& state : states)
        {
          // 使用 state.x_, state.y_
          // 偏移量
          State result_state = shiftUntilNoCollision(state, obstacles_points_, 0.1,2.0); 
          path[result_state.path_index_] = result_state;
        }
      }

      //在路径的两点之间进行S曲线的插值  位置平滑，速度连续，yaw自动算   使用五次多项式保证位置/速度/加速度连续
      if(region == FootprintRegion::LEFT_FRONT)
      {
        ROS_INFO("LEFT_FRONT");
        State left_front_end = states.back();
        int n = states.size();
        State left_front_end_start = states.front();
        int left_front_index = left_front_end_start.path_index_;
        std::vector<State> results = interpolateSCurve(path[left_front_index - 1], left_front_end, 0.1*n, n - 1);

        //修改路径点
        for (auto& state : results)
        {
          path[left_front_index] = state;
          left_front_index++;
        }
      }

      if(region == FootprintRegion::LEFT_REAR)
      {
        State left_rear_end = states.back();
        int n = states.size();
        State left_rear_start = states.front();
        int left_rear_index = left_rear_start.path_index_;
        std::vector<State> results = interpolateSCurve(path[left_rear_index - 1], left_rear_end, 0.1*n, n - 1);

        for (auto& state : results)
        {
          path[left_rear_index] = state;
          left_rear_index++;
        }
      }      

    }

    //发布修改后的路径点 path vector<State>;
    publishPathAsPointCloud(path_cloud_pub_,path);
  }else{ //检测到的每个路径点都平移10cm
  
  
  }
  return result;
}

DWAPlanner::State
DWAPlanner::shiftUntilNoCollision(
    DWAPlanner::State& original_path_point,
    const std::vector<geometry_msgs::Point>& obstacles,
    double step ,          // 10 cm
    double max_shift)      // 最大 ±60 cm
{
  for (double offset = 0.0; offset <= max_shift; offset += step)
  {
    // 向左
    auto left_shifted = ShiftPathLaterally(original_path_point, offset);
    ROS_INFO("%d",isPathPointInCollision(left_shifted, obstacles));
    if (!isPathPointInCollision(left_shifted, obstacles))
      return left_shifted;
    // 向右
    auto right_shifted = ShiftPathLaterally(original_path_point, -offset);
    if (!isPathPointInCollision(right_shifted, obstacles))
      return right_shifted;
  }

  return original_path_point; // 所有平移都失败
}

DWAPlanner::State DWAPlanner::ShiftPathLaterally(
    DWAPlanner::State& state,
    double lateral_offset)
{
  state.y_ += lateral_offset*sin(state.yaw_);
  state.x_ -= lateral_offset*cos(state.yaw_);
  return state;
}


std::vector<DWAPlanner::State> DWAPlanner::interpolateSCurve(
    const State& start,
    const State& end,
    double dt,
    int num_points)
{
  std::vector<State> result;

  SCurve1D sx, sy;

  sx.p0 = start.x_; sx.v0 = start.velocity_; sx.a0 = 0.0;
  sx.p1 = end.x_;   sx.v1 = end.velocity_;   sx.a1 = 0.0;
  sx.T = dt;

  sy.p0 = start.y_; sy.v0 = 0.0; sy.a0 = 0.0;
  sy.p1 = end.y_;   sy.v1 = 0.0; sy.a1 = 0.0;
  sy.T = dt;

  sx.compute();
  sy.compute();

  for (int i = 0; i <= num_points; ++i)
  {
    double t = dt * i / num_points;

    State s;
    s.x_ = sx.pos(t);
    s.y_ = sy.pos(t);
    s.yaw_ = atan2(sy.pos(t + 1e-3) - sy.pos(t),
                   sx.pos(t + 1e-3) - sx.pos(t));
    s.path_index_ = start.path_index_ + i + 1;
    result.push_back(s);
  }

  return result;
}

std::vector<DWAPlanner::Node*> DWAPlanner::AstartSearch(DWAPlanner::State start,DWAPlanner::State goal)
{

  std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open;
  std::unordered_set<int> closed;

  //速度和角速度的分辨率作为搜索的索引。
  const Window dynamic_window = calc_dynamic_window();

  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double yawrate_resolution =
      std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);

  Node* start_node = new Node{false,start, 0.0,heuristic(start, goal), {start},nullptr};  //

  auto makeKey = [&](int i, int j) { return i * velocity_samples_ + j; };
  open.push(start_node);


  
  std::map<std::pair<int, int>, std::pair<double, double>> velocity_map;
  for (int i = 0; i < velocity_samples_; i++)
  {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++)
    {
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;

      //i和j作为索引吗....
      //存储索引角速度和速度
      auto key = std::make_pair(i, j);
      velocity_map[key] = std::make_pair(v, y);

    }
  }


  // while (!open.empty())
  // {
  //   Node* current = open.top();
  //   open.pop();
  //   if (dist_to_goal_th_ > heuristic(start,goal)) //与目标点的距离在一定的范围内  //和目标路径小于一定的值
  //   {
  //     // 回溯路径
  //     std::vector<Node*> path;
  //     for (Node* n = current; n; n = n->parent)
  //       path.push_back(n);
  //     std::reverse(path.begin(), path.end());
  //     return path;
  //   }

  //   int key = current->lable;
  //   ROS_INFO("%d",key);
  //   if (closed.count(key)) continue;
  //   closed.insert(key);

  //   for (Node* nb : getNeighbors(current,velocity_resolution,yawrate_resolution))  //搜索附近的速度变化。  临近的有9个？
  //   {
  //     int nbKey = nb->lable; 
  //     if (closed.count(nbKey)) continue;

  //     double new_g = current->g + calc_obs_cost2(nb->state); //损失函数包含了 1.到目标点的dis，2.到障碍物的距离 3.
  //     if (new_g < nb->g)
  //     {
  //       nb->g = new_g;
  //       nb->h = heuristic(nb->state, goal);
  //       nb->parent = current;
  //       open.push(nb);
  //       nodes_[nbKey] = nb;
  //     }
  //   }
  // }
      ROS_INFO("end 3");
  return {};  // 无解
}


double DWAPlanner::heuristic(const DWAPlanner::State& start,const DWAPlanner::State& goal) const
{
  return std::hypot(start.x_ - goal.x_, start.y_ - goal.y_);
}

DWAPlanner::State DWAPlanner::motion_r(State &state_0, const double velocity, const double yawrate)
{
  State state = state_0;
  state.yaw_ += yawrate * sim_time_step; 
  state.x_ += velocity * std::cos(state.yaw_) * sim_time_step;
  state.y_ += velocity * std::sin(state.yaw_) * sim_time_step;
  state.velocity_ = velocity;
  state.yawrate_ = yawrate;
  state.path_index_++;
  return state;
}
//0.125 0.03
std::vector<DWAPlanner::Node*> DWAPlanner::getNeighbors(DWAPlanner::Node* node, double velocity_resolution,double yawrate_resolution)
{
  std::vector<Node*> neighbors;  //临近点，临近的速度和角速度？？
  std::initializer_list<double> dv_list = {
    -velocity_resolution,
    0.0,
    +velocity_resolution
  };
  std::initializer_list<double> dw_list = {
      -yawrate_resolution,
      0.0,
      +yawrate_resolution
  };
  // for (auto dv : dv_list){
  //   for (auto dw : dw_list){ 
  //     if (valid(node->state.velocity_ + dv, node->state.yawrate_ + dw))
  //     {
  //         ROS_INFO("Nodes vector size: %lf, %lf", node->state.velocity_ + dv ,node->state.yawrate_ + dw);
  //       // State state{v+dv,w+dw};//状态量的赋值 
  //       Node* nb = new Node; 
  //       nb->state = motion_r(node->state, node->state.velocity_ + dv, node->state.yawrate_ + dw);
  //       nb->g = node->g + calc_obs_cost2(nb->state);
  //       nb->h = calc_obs_cost2(nb->state);
  //       nb->parent = node;
  //       if(dv == -velocity_resolution)
  //       {
  //         nb->x = node->x - 1;
  //       }else if(dv == velocity_resolution){
  //         nb->x = node->x + 1;
  //       }else{
  //         nb->x = node->x;
  //       }
  //       if(dw == -yawrate_resolution)
  //       {
  //         nb->y = node->y - 1;
  //       }else if(dw == yawrate_resolution){
  //         nb->y = node->y + 1;
  //       }else{
  //         nb->y = node->y;  
  //       }
  //       neighbors.push_back(nb);
  //     }
  //   }
  // }

  ROS_INFO("Nodes vector size: %lu", neighbors.size());
  return neighbors;
}

bool DWAPlanner::valid(double v,double w)
{
  if(max_velocity_ >= v && v >= min_velocity_)
  {
    if(max_yawrate_ >= w && w >= min_yawrate_)
    {
      return true;
    }
  }
  return false;
}


float DWAPlanner::calc_obs_cost2(const State &state)
{
  float min_dist = obs_range_;
  for (const auto &obs : obs_list_.poses) //障碍物的点到足迹的最小距离。
  {
    float dist;
    if (use_footprint_)
      dist = calc_dist_from_robot(obs.position, state);
    else
      dist = hypot((state.x_ - obs.position.x), (state.y_ - obs.position.y)) - robot_radius_ - footprint_padding_; //可通行距离

    if (dist < DBL_EPSILON)
      return 1e6;
    min_dist = std::min(min_dist, dist);
  }
  
  return obs_range_ - min_dist;  
}


std::vector<DWAPlanner::State> DWAPlanner::AstartSearch2(DWAPlanner::State start,const Eigen::Vector3d  goal)
{

  
  std::unordered_set<int> closed;
  std::vector<DWAPlanner::State> path_points;

  //速度和角速度的分辨率作为搜索的索引。
  const Window dynamic_window = calc_dynamic_window();
  State end(goal[0],goal[1],0.0,0.0,0.0,0);

  Node* start_node = new Node{false,start, 0.0, heuristic(start, end), {start}, nullptr};  

  auto makeKey = [&](int i, int j) { return i * velocity_samples_ + j; };
  // open_.push(start_node);

  // Node* end_node = new Node{0,end, 0.0, heuristic(start, end), {end}, nullptr};  
  // open_end_.push(end_node);

  dwa_planning2(start_node,goal,heuristic(start_node->state,end));



  auto start_t = std::chrono::high_resolution_clock::now();
  // while (!open_.empty())//&&open_end_.empty()
  // {
  //   Node* current = open_.top();
  //   // Node* current_end = open_end_.top();

  //   open_.pop();
  //   // open_end_.pop();

  //   if (dist_to_goal_th_ > heuristic(current->state,end) || calc_predict_path_cost(current->path_points,end) < 0.1) //路径中的点和目标点小于阈值0.1.
  //   {
  //     //路径的长度必须大于原点到目标点的长度。
  //     if(calc_path_length(current->path_points) <= heuristic(start_node->state,end) + 0.2)
  //     {
  //       continue;
  //     }
  //     std::vector<State> path;
  //     path = current->path_points;
  //     ROS_INFO("Successfully obtained the path");
  //     return path;
  //   }

    
  //   if (closed.count(current->lable)) continue;
  //   closed.insert(current->lable);
    
  //   bool first_node = true;
  //   double last_h = 0;

  //   std::vector<DWAPlanner::Node*> tri = dwa_planning2(current,goal);
     
  //   // for (Node* nb : tri)  
  //   // {
  //   //   nb->g = heuristic(nb->state,end);
  //   //   // open_.push(nb);
  //   // }
  //   auto end_t = std::chrono::high_resolution_clock::now();
  //   auto duration_t = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
  //   if(duration_t > 60000)
  //   {
  //     break;
  //   }
  // }

  // while (!open_.empty())
  // {
  //     open_.pop();
  // }
  return {};  
}


bool DWAPlanner::dwa_planning2(DWAPlanner::Node* start,const Eigen::Vector3d goal,double dis)
{
  if (!start || start->visited) 
  {
    ROS_INFO("start->visited %d",start->path_points.size());
    best_traj_.first = start->path_points;
    trajectories_res_.push_back(best_traj_);
    return false;  
  }
  start->visited = true;
  State end(goal[0],goal[1],0.0,0.0,0.0,0);
  State begin(0.0,0.0,0.0,0.0,0.0,0);
  
  if(heuristic(start->state,end) > 1.2*heuristic(begin,end))
  {
    best_traj_.first = start->path_points;
    trajectories_res_.push_back(best_traj_);
    // visualize_trajectory(best_traj_.first, selected_trajectory_pub_);
    return false;
  }
  //路径偏离太远也要及时停止,实际转弯时还是需要1.57 的
  if(std::abs(start->state.yaw_) > 1.57 || start->path_points.size() > 70)
  {
    best_traj_.first = start->path_points;
    // visualize_trajectory(best_traj_.first, selected_trajectory_pub_);
    // ROS_INFO("start path point size %d",start->path_points.size());
    trajectories_res_.push_back(best_traj_);
    return false;
  }
  // std::vector<Node*> trajectories;
  // std::vector<std::pair<std::vector<State>, bool>> trajectories_res;


  if (dist_to_goal_th_ > heuristic(start->state,end) || calc_predict_path_cost(start->path_points,end) < 0.1) //路径中的点和目标点小于阈值0.1.
  {
    //路径的长度必须大于原点到目标点的长度。
    //到目标点的距离要小于上一个的
    if(calc_path_length(start->path_points) <= heuristic(begin,end) + 0.2) 
    {
      best_traj_.first = start->path_points;
      // visualize_trajectory(best_traj_.first, selected_trajectory_pub_);
      // ROS_INFO("start path point size %d",start->path_points.size());
      trajectories_res_.push_back(best_traj_);
      return false;
    }

    //路径中存在偏航角大于pi的
    std::vector<State> path;
    best_traj_.first = start->path_points;
    // best_traj_.first = start->path_points;
    trajectories_res_.push_back(best_traj_);
    // visualize_trajectory(best_traj_.first, selected_trajectory_pub_);
    ROS_INFO("Successfully obtained the path");
    return true;
  }


  // const Window dynamic_window = calc_dynamic_window_dynamic(start->state); //修改后存在问题 
  const Window dynamic_window = calc_dynamic_window(); //第一次不进行状态的处理。。。。 (0.5,0.0) 经过加速度的测算正好是最大最小值。

  std::vector<State> best_traj;
  best_traj.resize(sim_time_samples_);
  std::vector<Cost> costs;
  const size_t costs_size = velocity_samples_ * (yawrate_samples_ + 1); //速度3和角速度20的采样
  costs.reserve(costs_size);

  //速度和角速度的划分
  // ROS_INFO("dynamic_window.max_velocity_  = %lf,dynamic_window.min_velocity_ = %lf",dynamic_window.max_velocity_ ,dynamic_window.min_velocity_);

  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double yawrate_resolution =
      std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);
  // ROS_INFO("velocity_resolution = %lf,yawrate_resolution = %lf",velocity_resolution,yawrate_resolution);
  // int available_traj_count = 0; //有效的轨迹计数

  //从角速度为0向两边
  for (int i = 0; i < velocity_samples_; i++)
  {
    const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
    for (int j = 0; j < yawrate_samples_; j++)
    {
      std::pair<std::vector<State>, bool> traj;
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;
      if (v < slow_velocity_th_ && std::abs(y) < min_yawrate_)
      {
        continue;
      }

      traj.first = generate_trajectory2(start->state,v, y); 

      if (!calc_obs_cost3(traj.first))  
      {
        continue;
      }

      Node* nb = new Node; 
      nb->state = traj.first.back();
      nb->h = start->h + calc_obs_cost(traj.first) + calc_path_cost(traj.first); //路线点到障碍物的 = 上一个+当前的。 // 到目标点的
      nb->g = heuristic(nb->state,end);
      nb->parent = start;
      nb->path_points = start->path_points; // 循环累加重新赋值；  //第一次的+第二次的
      nb->path_points.insert(nb->path_points.end(),
                traj.first.begin(), traj.first.end());
      nb->visited = false;
      best_traj_.first = nb->path_points;
      visualize_trajectory(traj.first, selected_trajectory_pub_);

      std::pair<std::vector<State>, bool> best_traj;
      best_traj.first = nb->path_points; //所有的路径点均的足迹均不含障碍物点就是优的。
      best_traj.second = true;
            
      if (dwa_planning2(nb, goal,heuristic(start->state,end)))
      {
        return true; 
      }
      delete nb;   
      nb = nullptr;
          
    }

    if (dynamic_window.min_yawrate_ < 0.0 && 0.0 < dynamic_window.max_yawrate_)
    {
      std::pair<std::vector<State>, bool> traj;
      //1.计算轨迹
      traj.first = generate_trajectory2(start->state,v,start->state.yawrate_);  //state 为计算轨迹的最后一个状态量
      if (!calc_obs_cost3(traj.first))  
      {
        continue;
      }
      //2.评估
      Node* nb = new Node; 
      nb->state = traj.first.back(); //motion_r(start->state, start->state.velocity_ + v , start->state.yawrate_);
      nb->h = start->h + calc_obs_cost(traj.first)+calc_path_cost(traj.first); //路线点到障碍物的 = 上一个+当前的。 // 到目标点的
      nb->g = heuristic(nb->state,end);
      if(nb->g > start->g)
      {
        continue;
      }
      nb->parent = start;
      nb->path_points = traj.first; //start->path_points; // 循环累加重新赋值；  //第一次的+第二次的
      nb->path_points.insert(nb->path_points.end(),
                traj.first.begin(), traj.first.end());

      best_traj_.first = nb->path_points;
      visualize_trajectory(best_traj_.first, selected_trajectory_pub_);
      nb->visited = false;
            
      if(dwa_planning2(nb, goal,heuristic(start->state,end)))
        return true; 
      
      delete nb;
      nb = nullptr;
    }
  }
  // ROS_INFO("dwa planning2");

  // visualize_trajectories(trajectories_res, candidate_trajectories_pub_);  

  // return trajectories;
  return false;
}


std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory2(DWAPlanner::State state,const double velocity, const double yawrate)
{
  std::vector<State> trajectory;
  trajectory.resize(sim_time_samples_);
  for (int i = 0; i < sim_time_samples_; i++)
  {
    motion(state, velocity, yawrate);
    trajectory[i] = state;
  }
  return trajectory;
}

