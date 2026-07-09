// Copyright 2020 amsl

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    : local_nh_("~"), odom_updated_(false), local_map_updated_(false), scan_updated_(false), has_reached_(false),
      use_speed_cost_(false), odom_not_subscribe_count_(0), local_map_not_subscribe_count_(0),
      scan_not_subscribe_count_(0), vehicle_(1.67, 0.8, 0.52) 
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
  new_agv_path_cloud_pub_ = local_nh_.advertise<nav_msgs::Path>("new_agv_path", 10);

  // dist_to_goal_th_sub_ = nh_.subscribe("/dist_to_goal_th", 1, &DWAPlanner::dist_to_goal_th_callback, this);
  edge_on_global_path_sub_ = nh_.subscribe("/path", 1, &DWAPlanner::edge_on_global_path_callback, this); 
  control_data_sub_ = nh_.subscribe("/BsplinesController", 1, &DWAPlanner::ControlMsgCallback, this); 
  // footprint_sub_ = nh_.subscribe("/footprint", 1, &DWAPlanner::footprint_callback, this);
  // goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &DWAPlanner::goal_callback, this);
  // local_map_sub_ = nh_.subscribe("/local_map", 1, &DWAPlanner::local_map_callback, this);  不使用这个进行碰撞检测。
  // odom_sub_ = nh_.subscribe("/odom", 1, &DWAPlanner::odom_callback, this);
  // 速度为0时
  // scan_sub_ = nh_.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
  pointcloud2_sub1_ = nh_.subscribe("/pub_pointcloud1", 1, &DWAPlanner::cloudCallback1, this);  
  pointcloud2_sub2_ = nh_.subscribe("/pub_pointcloud2", 1, &DWAPlanner::cloudCallback2, this);  
  pointcloud2_sub3_ = nh_.subscribe("/pub_pointcloud3", 1, &DWAPlanner::cloudCallback3, this);  
  pointcloud2_sub4_ = nh_.subscribe("/pub_pointcloud4", 1, &DWAPlanner::cloudCallback4, this);  
  
  // target_velocity_sub_ = nh_.subscribe("/target_velocity", 1, &DWAPlanner::target_velocity_callback, this);

  sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_); //仿真步长 1s/10 = 0.01s
  cloud_pub_ =nh_.advertise<sensor_msgs::PointCloud2>("points", 1);

  if (use_footprint_)
  {
    footprint_ = vehicle_.makeFootprint();
    ROS_INFO("footprint_ %ld",footprint_.polygon.points.size());
    for (auto &point : footprint_.polygon.points)
    {
      point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
      point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
    }
  }
  obstacles_points_ = std::make_shared<std::vector<geometry_msgs::Point>>();
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
          global_frame_, ros::Time(0), goal_msg_.value(), goal_msg_.value().header.frame_id, goal_msg_.value()); //map坐标系下的。
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void DWAPlanner::ControlMsgCallback(const ControllerMsgPtr &controller_msg_ptr) {
    if(std::abs(controller_msg_ptr->Vx) < 0.1)
    {
      is_car_stop_ = true;
    }else{
      is_car_stop_ = false;
    }

    // ros_adapter_node::Controller2Camel_msg controller_msg;
    // controller_msg.id = controller_msg_ptr->id;
    // controller_msg.Vx = controller_msg_ptr->Vx;
    // controller_msg.Vy = controller_msg_ptr->Vy;
    // controller_msg.w = controller_msg_ptr->w;
    // controller_msg.status = controller_msg_ptr->status;
    // controller_msg.alarmCode = controller_msg_ptr->alarmCode;
    // controller_msg.od1 = controller_msg_ptr->od1;
    // controller_msg.od2 = controller_msg_ptr->od2;
    // controller_msg.indexFromEnd = controller_msg_ptr->indexFromEnd;
    // controller_msg.dataClearDone = controller_msg_ptr->dataClearDone;
    // controller_msg.autoObsAvoid = controller_msg_ptr->autoObsAvoid;
    // controller_msg.cir_Flag = controller_msg_ptr->cir_Flag;
    // controller_msg.terminal_status = controller_msg_ptr->terminal_status;
    // controller_msg.rotate_finished = controller_msg_ptr->rotate_finished;
    // controller_msg.typevw = controller_msg_ptr->typevw;
    // controller_msg.coord = controller_msg_ptr->coord;

    // new_controller_data_.push_back(controller_msg);
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
  // footprint_ = *msg; 
  // for (auto &point : footprint_.value().polygon.points)
  // {
  //   point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
  //   point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
  // }
  //足迹点为长方体：将足迹点划分成6个区域
}

void DWAPlanner::dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg)
{
  dist_to_goal_th_ = msg->data;
  ROS_INFO_STREAM_THROTTLE(1.0, "distance to goal threshold was updated to " << dist_to_goal_th_ << " [m]");
}

void DWAPlanner::edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg)
{
  if(msg->poses.empty())
  {
    return;
  }else{
    ROS_INFO("get  agv_path data");
  }
  if (!use_path_cost_)
    return;
  edge_points_on_path_ = *msg;
  edge_points_on_path_->header.frame_id = "map";
  try
  {
    int n = 0;
    local_path_length = 0.0;
    local_path_.clear();
    for (auto &pose : edge_points_on_path_.value().poses)
    {
      //local_path 本来就是base_link下的
      pose.header.frame_id = "map";
      pose.pose.position.x = pose.pose.position.x*0.001;
      pose.pose.position.y = pose.pose.position.y*0.001;
      listener_.transformPose(robot_frame_, ros::Time(0), pose, pose.header.frame_id, pose);

      //将路径转为states;
      State state;
      state.x_ = pose.pose.position.x;
      state.y_ = pose.pose.position.y;
      state.yaw_ = tf::getYaw(pose.pose.orientation);
      state.path_index_ = n++;
      local_path_.push_back(state);
    }
    int path_size = edge_points_on_path_.value().poses.size() - 1;
    //这个路径点在map坐标系下 距离当前base_link的位置
    for(int i = 0; i < path_size; i++)
    {
      geometry_msgs::PoseStamped pose1 = edge_points_on_path_.value().poses[i];
      geometry_msgs::PoseStamped pose2 = edge_points_on_path_.value().poses[i+1];
      local_path_length += std::sqrt((pose2.pose.position.x - pose1.pose.position.x)*(pose2.pose.position.x - pose1.pose.position.x)
       + (pose2.pose.position.y - pose1.pose.position.y)*(pose2.pose.position.y - pose1.pose.position.y));
    }
    ROS_INFO("local_path_length = %lf",local_path_length);
      
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  goal_msg_ = edge_points_on_path_.value().poses.back();
  goal_msg_.value().header.frame_id = "base_link";
  // if(goal_msg_.has_value())
  // {
  //   if (goal_msg_.value().header.frame_id != global_frame_)
  //   {
  //     try
  //     {
  //       // listener_.transformPose(
  //           // global_frame_, ros::Time(0), goal_msg_.value(), goal_msg_.value().header.frame_id, goal_msg_.value()); //map坐标系下的。
  //     }
  //     catch (tf::TransformException ex)
  //     {
  //       ROS_ERROR("%s", ex.what());
  //     }
  //   } 
  // }

}

void DWAPlanner::get_goal_msg()
{
  if(!edge_points_on_path_.has_value())
  {
    return;
  }
  for (auto &pose : edge_points_on_path_.value().poses)
  {
    //local_path 本来就是base_link下的
    pose.header.frame_id = "map";
    pose.pose.position.x = pose.pose.position.x*0.001;
    pose.pose.position.y = pose.pose.position.y*0.001;
    listener_.transformPose(robot_frame_, ros::Time(0), pose, pose.header.frame_id, pose);

    //将路径转为states;
    DWAPlanner::State state;
    state.x_ = pose.pose.position.x;
    state.y_ = pose.pose.position.y;
    state.yaw_ = tf::getYaw(pose.pose.orientation);
    float dis_2 = state.x_*state.x_ + state.y_*state.y_;
    ROS_INFO("x,y",state.x_,state.y_);
    if(dis_2 > 26)
    {
      goal_msg_ = pose;
      goal_msg_.value().header.frame_id = "base_link";
      break;
    }
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
    // get_goal_msg();
    // 对点云进行降采样
    auto  start_t = std::chrono::high_resolution_clock::now();
    obstacles_points_ = downsample(obstacles_points_,0.03);
    auto end_t = std::chrono::high_resolution_clock::now();
    auto duration_t = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
    double sec = duration_t / 1000.0;
    ROS_INFO("use time = %.3f s", sec); 
    if (can_move())
      cmd_vel = calc_cmd_vel();
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
    obstacles_points_->clear();

    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool DWAPlanner::can_move(void)
{
  // if (!footprint_.has_value())
  //   ROS_WARN_THROTTLE(1.0, "Robot Footprint has not been updated");
  if (!goal_msg_.has_value())
    ROS_WARN_THROTTLE(1.0, "Local goal has not been updated");
  if (!edge_points_on_path_.has_value())
    ROS_WARN_THROTTLE(1.0, "Edge on global path has not been updated");
  // if (subscribe_count_th_ < odom_not_subscribe_count_)
  //   ROS_WARN_THROTTLE(1.0, "Odom has not been updated");
  // if (subscribe_count_th_ < local_map_not_subscribe_count_)
  //   ROS_WARN_THROTTLE(1.0, "Local map has not been updated");
  // if (subscribe_count_th_ < scan_not_subscribe_count_)
    // ROS_WARN_THROTTLE(1.0, "Scan has not been updated");

  // if (!odom_updated_)
  //   odom_not_subscribe_count_++;
  // if (!local_map_updated_)
  //   local_map_not_subscribe_count_++;
  // if (!scan_updated_)
  //   scan_not_subscribe_count_++;

  if (edge_points_on_path_.has_value() && goal_msg_.has_value() )//&& !obstacles_points_->empty()&& is_car_stop_
  {
    if(local_path_length < 1.0)
    {
      ROS_INFO("local_path_length < 0.1, not can move");
      return false;
    }else{
      ROS_INFO("local_path_length > 0.1, can move");
      return true;
    }
    
  } //odom_not_subscribe_count_ <= subscribe_count_th_ ,footprint_.has_value() && goal_msg_.has_value()&& local_map_not_subscribe_count_ <= subscribe_count_th_ scan_not_subscribe_count_ <= subscribe_count_th_ 
  else
  {
    ROS_INFO("without agv_path or goal msg");
    return false;
  }
    
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
  ROS_INFO("goal_.pose.position.x %lf,goal_.pose.position.y%lf",goal_.pose.position.x,goal_.pose.position.y);
  
  //机器人当前位置到目标的平面的距离
  if (dist_to_goal_th_ < goal.segment(0, 2).norm() && !has_reached_)
  {
    //订阅存在障碍物的路径在障碍物的路径上进行搜索
    //障碍物的偏离量怎么计算
    std::pair<std::vector<State>, bool> traj;
    // dwa_planning_dynamic(current,goal,0,traj);
    auto start_t = std::chrono::high_resolution_clock::now();
    std::vector<State> path = AstarSearch2(current,goal);
    auto end_t = std::chrono::high_resolution_clock::now();
    auto duration_t2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();

    double sec = duration_t2 / 1000.0;
    ROS_INFO("use time = %.3f s", sec);

    for(auto p:path)
    {
      best_traj.first.push_back(p);
    }

    nav_msgs::Path new_path;
    new_path.header.frame_id = "map";
    // new_path.header.stamp = ros::Time::now();
    for(auto p:path)
    {
      geometry_msgs::PoseStamped pose;

      pose.header.frame_id = "base_link";
      pose.pose.position.x = p.x_;//1000
      pose.pose.position.y = p.y_;//1000  //发布一个新的new_agv_path;
      pose.pose.position.z = 0.0;

      // yaw 转四元数（单位：弧度）
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, p.yaw_); 
      pose.pose.orientation = tf2::toMsg(q);
      listener_.transformPose(global_frame_, ros::Time(0), pose, pose.header.frame_id, pose);
      pose.header.frame_id = "map";

      new_path.poses.push_back(pose);
    }
    new_agv_path_cloud_pub_.publish(new_path);


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
  // ROS_INFO("trajectories size = %ld",trajectories.size()); 

  visualize_trajectory(best_traj.first, selected_trajectory_pub_);


  // visualize_trajectories(trajectories4, candidate_trajectories_pub_);  

  // ROS_INFO("trajectories = %ld",trajectories.size()); 
  visualize_footprints(best_traj.first, predict_footprints_pub_);
  trajectories_results_.clear();

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
  Window window;  //-0.22~0.44
  window.min_velocity_ = 0.1;
  window.max_velocity_ = target_velocity_;
  window.min_yawrate_ = -0.5;
  window.max_yawrate_ = 0.5;
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
          ROS_INFO("state_point.x = %lf,state_point.y = %lf",state.x_ ,state.y_);
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
    //   dis += calc_dist_to_path(state);
    // }
    dis += calc_dist_to_path(traj.back());
  }
  return dis;
}

float DWAPlanner::calc_predict_path_cost(const std::vector<DWAPlanner::State> &traj,const DWAPlanner::State& goal,DWAPlanner::State &min_dis_state)
{
  double dis = FLT_MAX;
  if (!use_path_cost_)
    return 0.0;
  else
  {
    double last_dis = FLT_MAX;
    for(const auto& state:traj)
    {
      dis = std::min(heuristic(state,goal),dis);
      if(dis < last_dis)
      {
        min_dis_state = state;
      }
      last_dis = dis;
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
    footprint = footprint_;
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

// bool DWAPlanner::is_inside_of_robot(
//     const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped footprint, const State &state)
// {
//   geometry_msgs::Point32 state_point;
//   state_point.x = state.x_;
//   state_point.y = state.y_;


//   for (int i = 0; i < footprint.polygon.points.size(); i++)
//   {
//     geometry_msgs::Polygon triangle;
//     triangle.points.push_back(state_point);
//     triangle.points.push_back(footprint.polygon.points[i]);

//     if (i != footprint.polygon.points.size() - 1)
//       triangle.points.push_back(footprint.polygon.points[i + 1]);
//     else
//       triangle.points.push_back(footprint.polygon.points[0]);

//     if (is_inside_of_triangle(obstacle, triangle))
//       return true;
//   }

//   return false;
// }

bool DWAPlanner::is_inside_of_robot(const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped footprint, const State &state)
{
  if (footprint.polygon.points.empty())
    return false;

  // 1. 将障碍点转换到机器人坐标系
  // geometry_msgs::Point local_obs;
  // double cos_y = std::cos(-state.yaw_);
  // double sin_y = std::sin(-state.yaw_);

  // local_obs.x = (obstacle.x - state.x_) * cos_y -
  //               (obstacle.y - state.y_) * sin_y;
  // local_obs.y = (obstacle.x - state.x_) * sin_y +
  //               (obstacle.y - state.y_) * cos_y;
  // local_obs.z = 0.0;

  // 2. 射线法判断点是否在多边形内
  bool inside = false;
  const auto& pts = footprint.polygon.points;

  for (size_t i = 0; i < pts.size(); ++i)
  {
    size_t j = (i + 1) % pts.size();

    double xi = pts[i].x;
    double yi = pts[i].y;
    double xj = pts[j].x;
    double yj = pts[j].y;

    bool intersect =
        ((yi > obstacle.y) != (yj > obstacle.y)) &&
        (obstacle.x < (xj - xi) * (obstacle.y - yi) /
                       (yj - yi + 1e-9) + xi);

    if (intersect)
      inside = !inside;
  }

  return inside;
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
  obstacles_points_->clear();
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
    obstacles_points_->push_back(point);
    // ROS_INFO("obstacles_points_ %d", obstacles_points_.size());
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
  v_trajectory.lifetime = ros::Duration(60.0); 
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
    v_footprint.lifetime = ros::Duration(wait_time_); 
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

bool DWAPlanner::isPathPointInCollision2(
    const std::vector<DWAPlanner::State>& path,
    const std::vector<geometry_msgs::Point>& obstacles)
{
  int count = 0;
  for(auto s:path)
  {
    for (const auto& obs : obstacles)
    {
      const geometry_msgs::PolygonStamped footprint = move_footprint(s);
      if (is_inside_of_robot(obs,footprint,s))
      {
        return true;
      }
    }
  }
  return false;

  // if(count < 3)
  // {
  //   return false;
  // }else{
  //   return true;
  // }
  
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
    for (const auto& obs : *obstacles_points_)
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
    for (auto& [region, states] : result)
    {
      if(region == FootprintRegion::LEFT_MIDDLE)
      {
        for (auto& state : states)
        {
          // 使用 state.x_, state.y_
          // 偏移量
          State result_state = shiftUntilNoCollision(state, *obstacles_points_, 0.1,2.0); 
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

// std::vector<DWAPlanner::Node*> DWAPlanner::AstartSearch(DWAPlanner::State start,DWAPlanner::State goal)
// {

//   std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open;
//   std::unordered_set<int> closed;

//   //速度和角速度的分辨率作为搜索的索引。
//   const Window dynamic_window = calc_dynamic_window();

//   const double velocity_resolution =
//       std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
//   const double yawrate_resolution =
//       std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);

//   Node* start_node = new Node{0,start, 0.0,heuristic(start, goal), {start},nullptr};  //

//   auto makeKey = [&](int i, int j) { return i * velocity_samples_ + j; };
//   open.push(start_node);


  
//   std::map<std::pair<int, int>, std::pair<double, double>> velocity_map;
//   for (int i = 0; i < velocity_samples_; i++)
//   {
//     const double v = dynamic_window.min_velocity_ + velocity_resolution * i;
//     for (int j = 0; j < yawrate_samples_; j++)
//     {
//       double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;

//       //i和j作为索引吗....
//       //存储索引角速度和速度
//       auto key = std::make_pair(i, j);
//       velocity_map[key] = std::make_pair(v, y);

//     }
//   }


//   while (!open.empty())
//   {
//     Node* current = open.top();
//     open.pop();
//     if (dist_to_goal_th_ > heuristic(start,goal)) //与目标点的距离在一定的范围内  //和目标路径小于一定的值
//     {
//       // 回溯路径
//       std::vector<Node*> path;
//       for (Node* n = current; n; n = n->parent)
//         path.push_back(n);
//       std::reverse(path.begin(), path.end());
//       return path;
//     }

//     int key = current->lable;
//     ROS_INFO("%d",key);
//     if (closed.count(key)) continue;
//     closed.insert(key);

//     for (Node* nb : getNeighbors(current,velocity_resolution,yawrate_resolution))  //搜索附近的速度变化。  临近的有9个？
//     {
//       int nbKey = nb->lable; 
//       if (closed.count(nbKey)) continue;

//       double new_g = current->g + calc_obs_cost2(nb->state); //损失函数包含了 1.到目标点的dis，2.到障碍物的距离 3.
//       if (new_g < nb->g)
//       {
//         nb->g = new_g;
//         nb->h = heuristic(nb->state, goal);
//         nb->parent = current;
//         open.push(nb);
//         nodes_[nbKey] = nb;
//       }
//     }
//   }
//       ROS_INFO("end 3");
//   return {};  // 无解
// }


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


std::vector<DWAPlanner::State> DWAPlanner::AstarSearch2(DWAPlanner::State start,const Eigen::Vector3d  goal)
{  
  std::vector<DWAPlanner::State> path_points;
  State end(goal[0],goal[1],goal[2],0.0,0.0,0);
  Eigen::Vector2d A(goal[0],goal[1]);
  Eigen::Vector2d forward(cos(goal[2]),sin(goal[2]));
  Eigen::Vector2d A_dir = -forward;

  Node* start_node = new Node{0,start, 0.0, heuristic(start, end), {start}, nullptr};  

  while (!open_.empty())
  {
      open_.pop();
  }
  open_.push(start_node);

  auto start_t = std::chrono::high_resolution_clock::now();
  std::vector<State> path;
  //goal在start的后方
  if(goal[0] < 0)
  {
    target_velocity_ = -target_velocity_;
  }
  auto end_t = std::chrono::high_resolution_clock::now();
  while (!open_.empty())
  {
    Node* current = open_.top();
    open_.pop();

    double current_dis = heuristic(current->state,end);
    State min_dis_state;
    if (dist_to_goal_th_ > current_dis || calc_predict_path_cost(current->path_points,end,min_dis_state) < 0.3) //路径中的点和目标点小于阈值0.1.
    {
      //路径的长度必须大于原点到目标点的长度。
      if(calc_path_length(current->path_points) <= heuristic(start_node->state,end) + 0.2)
      {
        continue;
      }
      Eigen::Vector2d P0(current->parent->state.x_,current->parent->state.y_);
      //交点的位置不一定在前面，不要使用B曲线了 
      Eigen::Vector2d P1 = ControlPointP1(current->parent->state,end); 
      Eigen::Vector2d P2(goal[0],goal[1]);
      double dis = std::sqrt((P0.x() - P1.x())*(P0.x() - P1.x()) + (P0.y() - P1.y())*(P0.y() - P1.y()));
      std::vector<State> states;
      // states = smoothInterpolate(current->parent->state,end,1,10);
      // if(dis < 0.3)
      // // {
      states = smoothInterpolate(current->parent->state,end,1,10);
        // states = sampleBSpline(P0,P1,P2,5);
      // }else{
      //   states = sampleBSpline(P0,P1,P2,10);
      //   // states = sampleCubicBSpline(P0,P1,P2,10);
      // }
      // sensor_msgs::PointCloud2 cloud;
      // cloud.header.frame_id = "base_link";
      // cloud.header.stamp = ros::Time::now();
      // cloud.height = 1;
      // cloud.width  = 3;
      // cloud.is_bigendian = false;
      // cloud.is_dense = true;

      // cloud.fields.resize(3);
      // cloud.fields[0].name = "x";
      // cloud.fields[0].offset = 0;
      // cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      // cloud.fields[0].count = 1;

      // cloud.fields[1].name = "y";
      // cloud.fields[1].offset = 4;
      // cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      // cloud.fields[1].count = 1;

      // cloud.fields[2].name = "z";
      // cloud.fields[2].offset = 8;
      // cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      // cloud.fields[2].count = 1;

      // cloud.point_step = 12;
      // cloud.row_step = cloud.point_step * cloud.width;
      // cloud.data.resize(cloud.row_step);

      // float points[3][3] = {
      //     {P0.x(),P0.y(),0.0},
      //     {P1.x(),P1.y(),0.0},
      //     {P2.x(),P2.y(),0.0}
      // };

      // memcpy(&cloud.data[0], points, cloud.data.size());

      // cloud_pub_.publish(cloud);
      if(!isPathPointInCollision2(states, *obstacles_points_))
      {
        //路径点+插值的路径点。
        path = current->parent->path_points;
        path.insert(path.end(),
                  states.begin(), states.end());
        ROS_INFO("Successfully obtained the path dis < 0.1");
        auto duration_t = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
        return path;
      }else{
        path = current->path_points; 
        ROS_INFO("Successfully obtained the current->path_points");
        return path;
      }
    }

    //判断方向上是否存在交点  
    Eigen::Vector2d B(current->state.x_,current->state.y_);
    Eigen::Vector2d B_dir(cos(current->state.yaw_),sin(current->state.yaw_));
    Eigen::Vector2d dp ((B.x() - A.x()),(B.y()-A.y()));
    double cross = A_dir.x() * B_dir.y() - A_dir.y() * B_dir.x();
    double t = (dp.x()*A_dir.y() - dp.y()*A_dir.x()) / cross;
    double s = (dp.x()*B_dir.y() - dp.y()*B_dir.x()) / cross;
    if( t >= 0.1 && s >= 0.1)  
    {
      Eigen::Vector2d P0(current->state.x_,current->state.y_);
      Eigen::Vector2d P1 = ControlPointP1(current->state,end); //P1(B.x()+ t*B_dir.x(),B.y()+ t*B_dir.y());
      Eigen::Vector2d P2(goal[0],goal[1]);
      
      double dis = std::sqrt((P0.x() - P1.x())*(P0.x() - P1.x()) + (P0.y() - P1.y())*(P0.y() - P1.y()));
      std::vector<State> states;
      if(dis > 0.3)
      {
        states = sampleBSpline(P0,P1,P2,10);
                // states = sampleCubicBSpline(P0,P1,P2,10)
        if(!isPathPointInCollision2(states, *obstacles_points_))
        {
          //路径点+插值的路径点。
          path = current->path_points;
          ROS_INFO("Successfully obtained the path");
          path.insert(path.end(),
                    states.begin(), states.end());
          return path;
        }
        
      }

      // sensor_msgs::PointCloud2 cloud;
      // cloud.header.frame_id = "base_link";
      // cloud.header.stamp = ros::Time::now();
      // cloud.height = 1;
      // cloud.width  = 3;
      // cloud.is_bigendian = false;
      // cloud.is_dense = true;

      // cloud.fields.resize(3);
      // cloud.fields[0].name = "x";
      // cloud.fields[0].offset = 0;
      // cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      // cloud.fields[0].count = 1;

      // cloud.fields[1].name = "y";
      // cloud.fields[1].offset = 4;
      // cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      // cloud.fields[1].count = 1;

      // cloud.fields[2].name = "z";
      // cloud.fields[2].offset = 8;
      // cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      // cloud.fields[2].count = 1;

      // cloud.point_step = 12;
      // cloud.row_step = cloud.point_step * cloud.width;
      // cloud.data.resize(cloud.row_step);

      // float points[3][3] = {
      //     {P0.x(),P0.y(),0.0},
      //     {P1.x(),P1.y(),0.0},
      //     {P2.x(),P2.y(),0.0}
      // };

      // memcpy(&cloud.data[0], points, cloud.data.size());

      // cloud_pub_.publish(cloud);
      
      // visualize_trajectory(best_traj.first, selected_trajectory_pub_);
    }

    
    if (current->lable) 
    {
      continue;
    }
    current->lable = true;

    std::vector<DWAPlanner::Node*> tri = dwa_planning2(current,goal);

    for (Node* nb : tri)  
    {
      nb->g = heuristic(nb->state,end);
      open_.push(nb);
    }
    end_t = std::chrono::high_resolution_clock::now();
    auto duration_t = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
    if(duration_t > 10000)
    {
      ROS_INFO("time out");
      break;
    }
  }

  while (!open_.empty())
  {
      open_.pop();
  }
  return {};  
}


std::vector<DWAPlanner::Node*>
DWAPlanner::dwa_planning2(DWAPlanner::Node* start,const Eigen::Vector3d goal)
{
  std::vector<Node*> trajectories;
  std::vector<std::pair<std::vector<State>, bool>> trajectories_res;
  // const Window dynamic_window = calc_dynamic_window_dynamic(start->state); //修改后存在问题 
  float dis = std::sqrt((goal[0]-start->state.x_)*(goal[0]-start->state.x_) + (goal[1] - start->state.y_)*(goal[1] - start->state.y_));
  Window dynamic_window = calc_dynamic_window(); 

  const double velocity_resolution =
      std::max((dynamic_window.max_velocity_ - dynamic_window.min_velocity_) / (velocity_samples_ - 1), DBL_EPSILON);
  const double yawrate_resolution =
      std::max((dynamic_window.max_yawrate_ - dynamic_window.min_yawrate_) / (yawrate_samples_ - 1), DBL_EPSILON);

    const double v = target_velocity_;//dynamic_window.max_velocity_; 
    for (int j = 0; j < yawrate_samples_; j++)
    {
      std::pair<std::vector<State>, bool> traj;
      double y = dynamic_window.min_yawrate_ + yawrate_resolution * j;

      traj.first = generate_trajectory2(start->state,v, y);
      if(isPathPointInCollision2(traj.first, *obstacles_points_))
      {
        continue;
      }
      Node* nb = new Node; 
      nb->state = traj.first.back();
      nb->h =  calc_path_cost(traj.first); //start->h + calc_obs_cost(traj.first) + 路线点到障碍物的 = 上一个+当前的。 // 到目标点的
      nb->parent = start;
      nb->path_points = start->path_points; 
      for(auto p:traj.first)
      {
        nb->path_points.push_back(p);
      }
      std::pair<std::vector<State>, bool> best_traj;
      best_traj.first = nb->path_points; 
      best_traj.second = true;
      trajectories.push_back(nb);
      trajectories_res.push_back(best_traj);
    }
    visualize_trajectories(trajectories_res, candidate_trajectories_pub_);  

  return trajectories;
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

//生成原地旋转的点
std::vector<DWAPlanner::State> DWAPlanner::generate_trajectory3(DWAPlanner::State &state)
{
  std::vector<State> trajectory;
  
  for (int i = 0; i < 20; i++)
  {
    state.yaw_ += 0.3;

    const geometry_msgs::PolygonStamped footprint = move_footprint(state);
    bool is_inside = false;
    for (const auto& obs : *obstacles_points_)
    {
      if (is_inside_of_robot(obs,footprint,state))
      {
        is_inside = true;
        break;
      }
    }
    if(!is_inside)
    {
      trajectory.push_back(state); 
    }
  }
  return trajectory;
}


double DWAPlanner::getYawFromPose(const geometry_msgs::PoseStamped& pose)
{
    tf2::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;  // 弧度
}
// T:路径长度/期望速度
std::vector<DWAPlanner::State> DWAPlanner::smoothInterpolate(
    const DWAPlanner::State& start,
    const DWAPlanner::State& goal,
    double T,
    int N)
{
    std::vector<DWAPlanner::State> traj;
    double dx = goal.x_ - start.x_;
    double dy = goal.y_ - start.y_;
    double dyaw = goal.yaw_ - start.yaw_;

    // 角度归一化
    while (dyaw >  M_PI) dyaw -= 2*M_PI;
    while (dyaw < -M_PI) dyaw += 2*M_PI;

    for (int i = 0; i <= N; ++i)
    {
        double t = double(i) / N * T;
        double s = 10*pow(t/T,3)
                 -15*pow(t/T,4)
                 + 6*pow(t/T,5);

        DWAPlanner::State p;
        p.x_   = start.x_ + dx * s;
        p.y_   = start.y_ + dy * s;
        p.yaw_ = start.yaw_ + dyaw * s;
        traj.push_back(p);
    }
    return traj;
}

void DWAPlanner::quadraticBSplineInterp(
    const Eigen::Vector2d& Q0,
    const Eigen::Vector2d& Q1,
    const Eigen::Vector2d& Q2,
    Eigen::Vector2d& P0,
    Eigen::Vector2d& P1,
    Eigen::Vector2d& P2)
{
    Eigen::Matrix3d A;
    A << 1, 1, 0,
         1, 4, 1,
         0, 1, 1;

    Eigen::Vector3d Qx, Qy;
    Qx << 2*Q0.x(), 6*Q1.x(), 2*Q2.x();
    Qy << 2*Q0.y(), 6*Q1.y(), 2*Q2.y();

    Eigen::Vector3d Px = A.inverse() * Qx;
    Eigen::Vector3d Py = A.inverse() * Qy;

    P0 << Px(0), Py(0);
    P1 << Px(1), Py(1);
    P2 << Px(2), Py(2);
}

Eigen::Vector2d DWAPlanner::evalQuadraticBSpline(
    const Eigen::Vector2d& P0,
    const Eigen::Vector2d& P1,
    const Eigen::Vector2d& P2,
    double u)   // u ∈ [0,1]
{
    double N0 = (1-u)*(1-u) / 2.0;
    double N1 = (-2*u*u + 2*u + 1) / 2.0;
    double N2 = u*u / 2.0;

    return N0*P0 + N1*P1 + N2*P2;
}
/*
Q0:起点
Q1:中间点
Q2:终点
N:10~20，6~10.

*/
std::vector<DWAPlanner::State> DWAPlanner::generateTrajectory(
    const Eigen::Vector2d& Q0,
    const Eigen::Vector2d& Q1,
    const Eigen::Vector2d& Q2,
    int N)
{
    Eigen::Vector2d P0,P1,P2;
    quadraticBSplineInterp(Q0,Q1,Q2,P0,P1,P2);

    std::vector<Eigen::Vector2d> traj;
    for (int i=0;i<N;++i){
        double u = double(i)/(N-1);
        traj.push_back(evalQuadraticBSpline(P0,P1,P2,u));
    }

    //vector 转 state
    std::vector<DWAPlanner::State> res;
    for(auto t:traj)
    {
      DWAPlanner::State state;
      state.x_ = t.x();
      state.y_ = t.y();
      res.push_back(state);
    }
    return res;
}

std::vector<DWAPlanner::State> DWAPlanner::sampleBSpline(   
    const Eigen::Vector2d& P0,
    const Eigen::Vector2d& P1,
    const Eigen::Vector2d& P2,int N) {
      // std::vector<Eigen::Vector2d> controlPoints;
      // controlPoints.push_back(P0);
      // controlPoints.push_back(P1);
      // controlPoints.push_back(P2);
      // return sampleBSpline2(controlPoints,N);
    std::vector<Eigen::Vector3d> traj;
    for (int i = 0; i < N; ++i) {
        double t = static_cast<double>(i) / (N - 1);
        Eigen::Vector2d pt = (1.0 - t) * (1.0 - t) * P0 
                   + 2.0 * t * (1.0 - t) * P1
                   + t * t * P2;
        Eigen::Vector2d pv = 2*((t-1.0) *P0 + (1-2*t)*P1 + 2*t*P2);
        double yaw = std::atan2(pv.y(),pv.x());
        Eigen::Vector3d p(pt.x(),pt.y(),yaw);
        traj.push_back(p);
    }

    std::vector<DWAPlanner::State> res;
    for(auto t:traj)
    {
      DWAPlanner::State state;
      state.x_ = t.x();
      state.y_ = t.y();
      state.yaw_ = t.z();
      res.push_back(state);
    }

    return res;
}


std::vector<geometry_msgs::Point> pointCloudToVector(
    const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    std::vector<geometry_msgs::Point> pts;
    pts.reserve(cloud->width);

    const uint8_t* data = cloud->data.data();
    const int step = cloud->point_step;

    for (sensor_msgs::PointCloud2ConstIterator<float>
        iter_x(*cloud, "x"),
        iter_y(*cloud, "y"),
        iter_z(*cloud, "z");
        iter_x != iter_x.end();
        ++iter_x, ++iter_y, ++iter_z)
    {
      geometry_msgs::Point p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      pts.push_back(p);
    }

    return pts;
}

void DWAPlanner::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> obstacles_points;
  obstacles_points = pointCloudToVector(msg);
  obstacles_points_->insert(obstacles_points_->end(),
                    obstacles_points.begin(), obstacles_points.end());
  if(obstacles_points_->empty())
  {
    ROS_INFO("obstacles_points_ is empty;");
  }
}

void DWAPlanner::cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> obstacles_points;
  State state;
  obstacles_points = pointCloudToVector(msg);
  for (const auto& obs : obstacles_points)
  {
    if (is_inside_of_robot(obs,footprint_,state))
    {
      continue;
    }
    obstacles_points_->push_back(obs);
  }
  if(obstacles_points_->empty())
  {
    // ROS_INFO("obstacles_points_ is empty;");
  }
}
void DWAPlanner::cloudCallback3(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> obstacles_points;
  State state;
  obstacles_points = pointCloudToVector(msg);
  for (const auto& obs : obstacles_points)
  {
    if (is_inside_of_robot(obs,footprint_,state))
    {
      continue;
    }
    obstacles_points_->push_back(obs);
  }
  if(obstacles_points_->empty())
  {
    // ROS_INFO("obstacles_points_ is empty;");
  }
}
void DWAPlanner::cloudCallback4(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::vector<geometry_msgs::Point> obstacles_points;
  State state;
  obstacles_points = pointCloudToVector(msg);
  for (const auto& obs : obstacles_points)
  {
    if (is_inside_of_robot(obs,footprint_,state))
    {
      continue;
    }
    obstacles_points_->push_back(obs);
  }
    
  if(obstacles_points_->empty())
  {
    // ROS_INFO("obstacles_points_ is empty;");
  }
}


double DWAPlanner::bSplineBasis(int i, int p, double t,
                    const std::vector<double>& knots) {
    if (p == 0) {
        return (knots[i] <= t && t < knots[i+1]) ? 1.0 : 0.0;
    }
    double left = 0.0, right = 0.0;
    if (knots[i+p] != knots[i])
        left = (t - knots[i]) /
               (knots[i+p] - knots[i]) *
               bSplineBasis(i, p-1, t, knots);
    if (knots[i+p+1] != knots[i+1])
        right = (knots[i+p+1] - t) /
                (knots[i+p+1] - knots[i+1]) *
                bSplineBasis(i+1, p-1, t, knots);
    return left + right;
}

std::vector<DWAPlanner::State>
DWAPlanner::sampleBSpline2(
    const std::vector<Eigen::Vector2d>& controlPoints,
    int N)
{
    const int p = 3; // cubic
    std::vector<double> knots;

    // clamped uniform knot vector
    for (int i = 0; i <= controlPoints.size() + p; ++i) {
        if (i < p) knots.push_back(0.0);
        else if (i <= controlPoints.size())
            knots.push_back(static_cast<double>(i - p) /
                            (controlPoints.size() - p));
        else knots.push_back(1.0);
    }

    std::vector<DWAPlanner::State> traj;
    for (int i = 0; i < N; ++i) {
        double t = static_cast<double>(i) / (N - 1);

        Eigen::Vector2d pt(0, 0);
        for (size_t j = 0; j < controlPoints.size(); ++j) {
            double B = bSplineBasis(j, p, t, knots);
            pt += B * controlPoints[j];
        }

        traj.emplace_back();
        traj.back().x_ = pt.x();
        traj.back().y_ = pt.y();
    }

    // yaw from finite difference (smooth now!)
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i == 0) {
            traj[i].yaw_ = std::atan2(
                traj[1].y_ - traj[0].y_,
                traj[1].x_ - traj[0].x_);
        } else {
            traj[i].yaw_ = std::atan2(
                traj[i].y_ - traj[i-1].y_,
                traj[i].x_ - traj[i-1].x_);
        }
    }

    return traj;
}

std::vector<DWAPlanner::State>
DWAPlanner::sampleCubicBSpline(
    const Eigen::Vector2d& P0,
    const Eigen::Vector2d& P1,
    const Eigen::Vector2d& P2,
    int N)
{
    // 1. 构造 4 个控制点
    std::vector<Eigen::Vector2d> ctrl;
    ctrl.push_back(P0);
    ctrl.push_back(P1);
    ctrl.push_back(P2);
    ctrl.push_back(P2);  // 重复终点

    auto basis = [](double u) -> Eigen::Vector4d {
        double u2 = u * u;
        double u3 = u2 * u;
        return Eigen::Vector4d(
            (1 - u) * (1 - u) * (1 - u) / 6.0,
            (3*u3 - 6*u2 + 4) / 6.0,
            (-3*u3 + 3*u2 + 3*u + 1) / 6.0,
            u3 / 6.0
        );
    };

    std::vector<DWAPlanner::State> res;

    for (int i = 0; i < N; ++i) {
        double u = static_cast<double>(i) / (N - 1);

        Eigen::Vector4d B = basis(u);
        Eigen::Vector2d pt(0, 0);

        for (int j = 0; j < 4; ++j)
            pt += B[j] * ctrl[j];

        // 数值差分算 yaw
        double eps = 1e-3;
        Eigen::Vector4d Bp = basis(std::min(u + eps, 1.0));
        Eigen::Vector2d pt_next(0, 0);
        for (int j = 0; j < 4; ++j)
            pt_next += Bp[j] * ctrl[j];

        Eigen::Vector2d v = pt_next - pt;
        double yaw = std::atan2(v.y(), v.x());

        DWAPlanner::State s;
        s.x_ = pt.x();
        s.y_ = pt.y();
        s.yaw_ = yaw;
        res.push_back(s);
    }

    return res;
}

Eigen::Vector2d DWAPlanner::ControlPointP1(DWAPlanner::State &s1, DWAPlanner::State &s2)
{
    //判断方向上是否存在交点
    Eigen::Vector2d A(s1.x_,s1.y_);
    Eigen::Vector2d A_dir(cos(s1.yaw_),sin(s1.yaw_));
    Eigen::Vector2d B(s2.x_,s2.y_);
    Eigen::Vector2d B_dir(cos(s2.yaw_),sin(s2.yaw_));
    B_dir = -B_dir;
    Eigen::Vector2d dp ((B.x() - A.x()),(B.y()-A.y()));
    double cross = A_dir.x() * B_dir.y() - A_dir.y() * B_dir.x();
    double t = (dp.x()*A_dir.y() - dp.y()*A_dir.x()) / cross;
    double s = (dp.x()*B_dir.y() - dp.y()*B_dir.x()) / cross;

    Eigen::Vector2d P1(B.x()+ t*B_dir.x(),B.y()+ t*B_dir.y());

    return P1;
}


std::shared_ptr<std::vector<geometry_msgs::Point>> DWAPlanner::downsample(
    const std::shared_ptr<const std::vector<geometry_msgs::Point>>& pts,
    double resolution)
{
  std::unordered_set<DWAPlanner::VoxelKey, DWAPlanner::VoxelHash> seen;
  seen.reserve(pts->size());
  auto out = std::make_shared<std::vector<geometry_msgs::Point>>();

  for (auto p : *pts) {
    p.z = 0;
    DWAPlanner::VoxelKey k{
      static_cast<int>(std::floor(p.x / resolution)),
      static_cast<int>(std::floor(p.y / resolution)),
      static_cast<int>(std::floor(p.z / resolution))
    };

    if (seen.insert(k).second)
      out->push_back(p);
  }
  return out;
}
