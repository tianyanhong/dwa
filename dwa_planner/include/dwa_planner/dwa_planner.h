// Copyright 2020 amsl

/**
 * @file dwa_plannr.h
 * @brief C++ implementation for dwa planner
 * @author AMSL
 */

#ifndef DWA_PLANNER_DWA_PLANNER_H
#define DWA_PLANNER_DWA_PLANNER_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>

#include <thread>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cstring>

#include "ros_adapter_node/Controller2Camel_msg.h"

/**
 * @class DWAPlanner
 * @brief A class implementing a local planner using the Dynamic Window Approach
 */
class DWAPlanner
{
public:
  /**
   * @brief Constructor for the DWAPlanner
   */
  DWAPlanner(void);

typedef ros_adapter_node::Controller2Camel_msg::ConstPtr ControllerMsgPtr;
struct VehicleParams
{
  double length;       // 车长
  double width;        // 车宽
  double rear_offset;  // 运动中心到车尾距离

  VehicleParams() = default;

  VehicleParams(double l, double w, double offset)
    : length(l), width(w), rear_offset(offset) {}

  geometry_msgs::PolygonStamped makeFootprint() const
  {
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    footprint.header.stamp = ros::Time::now();

    geometry_msgs::Point32 p1;
    geometry_msgs::Point32 p2;
    geometry_msgs::Point32 p3;
    geometry_msgs::Point32 p4;

    const double front = length - rear_offset;
    const double rear  = -rear_offset;
    const double half_w = 0.5 * width;

    p1.x = front;
    p1.y = half_w;
    p1.z = 0;

    p2.x = front;
    p2.y = -half_w;
    p2.z = 0;

    p3.x = rear;
    p3.y = -half_w;
    p3.z = 0;

    p4.x = rear;
    p4.y = half_w;
    p4.z = 0;

    footprint.polygon.points.push_back(p1);
    footprint.polygon.points.push_back(p2);
    footprint.polygon.points.push_back(p3);
    footprint.polygon.points.push_back(p4);
    return footprint;
  }
};

struct VoxelKey {
  int x, y, z;

  bool operator==(const VoxelKey& o) const {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct VoxelHash {
  std::size_t operator()(const VoxelKey& k) const {
    return ((std::uint64_t)k.x << 40) ^
           ((std::uint64_t)k.y << 20) ^
           ((std::uint64_t)k.z);
  }
};

enum class FootprintRegion
{
  LEFT_FRONT,
  LEFT_MIDDLE,
  LEFT_REAR,
  RIGHT_FRONT,
  RIGHT_MIDDLE,
  RIGHT_REAR,
  UNKNOWN,
};




struct SCurve1D
{
  double p0, v0, a0;
  double p1, v1, a1;
  double T;

  std::array<double, 6> coeffs;

  void compute()
  {
    coeffs[0] = p0;
    coeffs[1] = v0;
    coeffs[2] = a0 / 2.0;

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    Eigen::Matrix<double, 3, 3> A;
    A << T3,   T4,   T5,
         3*T2, 4*T3, 5*T4,
         6*T, 12*T2, 20*T3;

    Eigen::Vector3d b;
    b << p1 - (p0 + v0*T + a0*T2/2.0),
         v1 - (v0 + a0*T),
         a1 - a0;

    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

    coeffs[3] = x[0];
    coeffs[4] = x[1];
    coeffs[5] = x[2];
  }

  double pos(double t) const
  {
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t*t +
           coeffs[3]*t*t*t + coeffs[4]*t*t*t*t +
           coeffs[5]*t*t*t*t*t;
  }
};


  /**
   * @class State
   * @brief A data class for state of robot
   */
  class State
  {
  public:
    /**
     * @brief Constructor
     */
    State(void);

    /**
     * @brief Constractor
     * @param x The x position of robot
     * @param y The y position of robot
     * @param yaw The orientation of robot
     * @param velocity The linear velocity of robot
     * @param yawrate The angular velocity of robot
     */
    State(const double x, const double y, const double yaw, const double velocity, const double yawrate,const int path_index);

    double x_;
    double y_;
    double yaw_;
    double velocity_;
    double yawrate_;
    int path_index_; 

  private:
  };


  using CollisionPointsByRegion = std::unordered_map<FootprintRegion, std::vector<State>>;

  /**
   * @class Window
   * @brief A data class for dynamic window
   */
  class Window
  {
  public:
    /**
     * @brief Constructor
     */
    Window(void);

    /**
     * @brief Show the dynamic window information
     */
    void show(void);

    double min_velocity_;
    double max_velocity_;
    double min_yawrate_;
    double max_yawrate_;

  private:
  };

  /**
   * @class Cost
   * @brief A data class for cost
   */
  class Cost
  {
  public:
    /**
     * @brief Constructor
     */
    Cost(void);

    /**
     * @brief Constructor
     * @param obs_cost The cost of obstacle
     * @param to_goal_cost The cost of distance to goal
     * @param speed_cost The cost of speed
     * @param path_cost The cost of path
     * @param total_cost The total cost
     */
    Cost(
        const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
        const float total_cost);

    /**
     * @brief Show the cost
     */
    void show(void);

    /**
     * @brief Calculate the total cost
     */
    void calc_total_cost(void);

    float obs_cost_;
    float to_goal_cost_;
    float speed_cost_;
    float path_cost_;
    float total_cost_;

  private:
  };

    //执行A*的相关数据结构
  struct Node
  {
    bool lable = false;
    State state;
    double g, h;  //g是到目标点的 h是距离的。
    std::vector<State> path_points;
    Node* parent;
    //路径点

    double f() const { return g; } //+ h
  };

  struct NodeComparator
  {
    bool operator()(const Node* a, const Node* b) const
    {
      return a->f() > b->f();  // 小顶堆
    }
  };

  /**
   * @brief Execute local path planning
   */
  void process(void);

  /**
   * @brief Load parameters
   */
  void load_params(void);

  /**
   * @brief Print parameters
   */
  void print_params(void);

  /**
   * @brief A callback to hanldle buffering local goal messages
   */
  void goal_callback(const geometry_msgs::PoseStampedConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering scan messages
   */
  void scan_callback(const sensor_msgs::LaserScanConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering local map messages
   */
  void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering odometry messages
   */
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief A callback to hanldle buffering target velocity messages
   */
  void target_velocity_callback(const geometry_msgs::TwistConstPtr &msg);

  /**
   * @brief A calllback to handle buffering footprint messages
   */
  void footprint_callback(const geometry_msgs::PolygonStampedPtr &msg);

  /**
   * @brief A callback to handle buffering distance to goal threshold messages
   */
  void dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg);

  /**
   * @brief A callback to handle buffering edge on global path messages
   */
  void edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg);

  /**
   * @brief Calculate dynamic window
   * @return The dynamic window
   */
  Window calc_dynamic_window(void);
  Window calc_dynamic_window_dynamic(const State current);

  /**
   * @brief Calculate obstacle cost
   * @param traj The estimated trajectory
   * @return The obstacle cost
   */
  float calc_obs_cost(const std::vector<State> &traj);

  /**
   * @brief Calculate the distance of current pose to goal pose
   * @param traj The estimated trajectory
   * @param goal The pose of goal
   * @return The distance of current pose to goal pose
   */
  float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);

  /**
   * @brief Calculate the speed cost
   * @param traj The estimated trajectory
   * @return The speed cost
   */
  float calc_speed_cost(const std::vector<State> &traj);

  /**
   * @brief Calculate the path cost
   * @param traj The estimated trajectory
   * @return The path cost
   */
  float calc_path_cost(const std::vector<State> &traj);

  /**
   * @brief Calculate the distance of current pose to global path
   * @param state The robot state
   * @return The distance of current pose to global path
   */
  float calc_dist_to_path(const State state);

  /**
   * @brief Simulate the robot motion
   * @param state The start state of robot
   * @param velocity The velocity of robot
   * @param yawrate The angular velocity of robot
   */
  void motion(State &state, const double velocity, const double yawrate);

  /**
   * @brief Get obstacle list from local map
   * @param map The local map
   */
  void create_obs_list(const nav_msgs::OccupancyGrid &map);

  /**
   * @brief Get obstacle list from laser scan
   * @param scan The laser scan
   */
  void create_obs_list(const sensor_msgs::LaserScan &scan);

  /**
   * @brief Calculate the distance from robot footprint to the nearest obstacle
   * @param obstacle The position of obstacle
   * @param state The robot state
   * @return The distance from robot footprint to the nearest obstacle
   */
  float calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state);

  /**
   * @brief Move the robot footprint to the target pose
   * @param target_pose The target pose
   * @return The moved footprint
   */
  geometry_msgs::PolygonStamped move_footprint(const State &target_pose);

  /**
   * @brief Check if the obstacle is inside of robot footprint
   * @param obstacle The position of obstacle
   * @param footprint The robot footprint
   * @param state The robot state
   * @return True if the obstacle is inside of robot footprint
   */
  bool is_inside_of_robot(
      const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped footprint, const State &state);

  /**
   * @brief Check if the target point is inside of triangle
   * @param target_point The target point
   * @param triangle The triangle
   * @return True if the target point is inside of triangle
   */
  bool is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle);

  /**
   * @brief Calculate the intersection point of the line and the circle
   * @param obstacle The position of obstacle
   * @param state The robot state
   * @param footprint The robot footprint
   * @return The intersection point of the line and the circle
   */
  geometry_msgs::Point
  calc_intersection(const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint);

  /**
   * @brief Generate trajectory
   * @param velocity The velocity of robot
   * @param yawrate The angular velocity of robot
   * @return The generated trajectory
   */
  std::vector<State> generate_trajectory(const double velocity, const double yawrate);

  /**
   * @brief Generate trajectory
   * @param yawrate The angular velocity of robot
   * @param goal The pose of goal
   * @return The generated trajectory
   */
  std::vector<State> generate_trajectory(const double yawrate, const Eigen::Vector3d &goal);


  std::vector<State> generate_trajectory_with_depth(const double velocity, const double yawrate,const Eigen::Vector3d &goal, int depth);


  /**
   * @brief Evaluate trajectory
   * @param trajectory The estimated trajectory
   * @param goal The pose of goal
   * @return The cost of trajectory
   */
  Cost evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal);

  /**
   * @brief Check if the robot can move
   * @return True if the robot can move
   */
  bool can_move(void);

  /**
   * @brief Calculate the command velocity
   * @return The command velocity
   */
  geometry_msgs::Twist calc_cmd_vel(void);

  /**
   * @brief Check if the robot can adjust the direction
   * @param goal The pose of goal
   * @return True if the robot can adjust the direction
   */
  bool can_adjust_robot_direction(const Eigen::Vector3d &goal);

  /**
   * @brief Check if the robot has collided
   * @param traj The estimated trajectory
   * @return True if the robot has collided
   */
  bool check_collision(const std::vector<State> &traj);

  /**
   * @brief Normalize the costs
   * @param costs array of costs
   */
  void normalize_costs(std::vector<Cost> &costs);

  /**
   * @brief Create a marker message
   * @param id The id of marker
   * @param scale The scale of marker
   * @param color The color of marker
   * @param trajectory The estimated trajectory
   * @param footprint The robot footprint
   */
  visualization_msgs::Marker create_marker_msg(
      const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
      const geometry_msgs::PolygonStamped &footprint = geometry_msgs::PolygonStamped());

  /**
   * @brief Publish selected trajectory
   * @param trajectory Selected trajectry
   * @param pub Publisher of selected trajectory
   */
  void visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub);

  /**
   * @brief Publish candidate trajectories
   * @param trajectories Candidated trajectories
   * @param pub Publisher of candidate trajectories
   */
  void visualize_trajectories(
      const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub);

  /**
   * @brief Publish predicted footprints
   * @param trajectory Selected trajectry
   * @param pub Publisher of predicted footprints
   */
  void visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub);

  /**
   * @brief Execute dwa planning
   * @param window Dynamic window
   * @param goal Goal pose
   * @param obs_list Obstacle's position
   */
  std::vector<State>
  dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories);

  void dwa_planning_dynamic(const State& current, const Eigen::Vector3d &goal, int depth,std::pair<std::vector<State>, bool> &traj);

  void publishPathAsPointCloud(
    ros::Publisher& pub,
    const std::vector<State>& states);
  double calc_path_length(const std::vector<State>& traj);

  void searchPath(const State& current, const Eigen::Vector3d goal, int depth);

  FootprintRegion getRegion(
      const geometry_msgs::Point& point,
      const VehicleParams& robot);

  bool isPathPointInCollision(
      const State& state,
      const std::vector<geometry_msgs::Point>& obstacles);

  std::vector<State> selectCollidingPathPoints(
      const std::vector<State>& path,
      const std::vector<geometry_msgs::Point>& obstacles);

  CollisionPointsByRegion classifyCollidingPointsByRegion(
      std::vector<State>& path,
      const VehicleParams& vehicle,
      const std::vector<geometry_msgs::Point>& obstacles);

  State shiftUntilNoCollision(
      State& original_path_point,
      const std::vector<geometry_msgs::Point>& obstacles,
      double step = 0.10,          // 10 cm
      double max_shift = 0.6); 

  State ShiftPathLaterally(
    State& path,
    double lateral_offset);

  std::vector<State> interpolateSCurve(
      const State& start,
      const State& end,
      double dt,
      int num_points);
  // std::vector<Node*> AstartSearch(State start,State goal);
  double heuristic(const State& start,const State& end) const;
  bool isInsidePolygon(const geometry_msgs::Point& p,
                     const geometry_msgs::PolygonStamped& poly);
  State motion_r(State &state_0, const double velocity, const double yawrate);
  std::vector<Node*> getNeighbors(Node* node, double velocity_resolution,double yawrate_resolution);
  bool valid(double v,double w);
  float calc_obs_cost2(const State &state);

  std::vector<State> AstarSearch2(State start,const Eigen::Vector3d goal);

  std::vector<Node*>
  dwa_planning2(Node* start,const Eigen::Vector3d goal);

  std::vector<State> generate_trajectory2(State state,const double velocity, const double yawrate);
  float calc_predict_path_cost(const std::vector<State> &traj,const State& goal,State &min_dis_state);
  bool calc_obs_cost3(const std::vector<State> &traj);
  double getYawFromPose(const geometry_msgs::PoseStamped& pose);
  std::vector<State> smoothInterpolate(
    const State& start,
    const State& goal,
    double T,
    int N );
  bool isPathPointInCollision2(
      const std::vector<State>& path,
      const std::vector<geometry_msgs::Point>& obstacles);
  void quadraticBSplineInterp(
      const Eigen::Vector2d& Q0,
      const Eigen::Vector2d& Q1,
      const Eigen::Vector2d& Q2,
      Eigen::Vector2d& P0,
      Eigen::Vector2d& P1,
      Eigen::Vector2d& P2);
  Eigen::Vector2d evalQuadraticBSpline(
      const Eigen::Vector2d& P0,
      const Eigen::Vector2d& P1,
      const Eigen::Vector2d& P2,
      double u);
  std::vector<State>  generateTrajectory(
      const Eigen::Vector2d& Q0,
      const Eigen::Vector2d& Q1,
      const Eigen::Vector2d& Q2,
      int N);
  std::vector<State> sampleBSpline(   
      const Eigen::Vector2d& Q0,
      const Eigen::Vector2d& Q1,
      const Eigen::Vector2d& Q2,int N); 
  void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudCallback3(const sensor_msgs::PointCloud2ConstPtr& msg);
  void cloudCallback4(const sensor_msgs::PointCloud2ConstPtr& msg);

  std::vector<State> generate_trajectory3(State &state);
  
  double bSplineBasis(int i, int p, double t,
                    const std::vector<double>& knots);

  std::vector<State> sampleBSpline2(const std::vector<Eigen::Vector2d>& controlPoints, int N);
  std::vector<State> sampleCubicBSpline(const Eigen::Vector2d& P0, const Eigen::Vector2d& P1, const Eigen::Vector2d& P2,int N);
  Eigen::Vector2d ControlPointP1(State &s1, State &s2);
  void get_goal_msg();
  void ControlMsgCallback(const ControllerMsgPtr &controller_msg_ptr);
  std::shared_ptr<std::vector<geometry_msgs::Point>> downsample(
    const std::shared_ptr<const std::vector<geometry_msgs::Point>>& pts,
      double resolution);
protected:
  std::string global_frame_;
  std::string robot_frame_;
  double hz_;
  double target_velocity_;
  double max_velocity_;
  double min_velocity_;
  double max_yawrate_;
  double min_yawrate_;
  double max_in_place_yawrate_;
  double min_in_place_yawrate_;
  double max_acceleration_;
  double max_deceleration_;
  double max_d_yawrate_;
  double sim_period_;
  double angle_resolution_;
  double predict_time_;
  double sleep_time_after_finish_;
  double obs_cost_gain_;
  double to_goal_cost_gain_;
  double speed_cost_gain_;
  double path_cost_gain_;
  double dist_to_goal_th_;
  double turn_direction_th_;
  double angle_to_goal_th_;
  double sim_direction_;
  double slow_velocity_th_;
  double obs_range_;
  double robot_radius_;
  double footprint_padding_;
  double v_path_width_;
  bool use_footprint_;
  bool use_scan_as_input_;
  bool use_path_cost_;
  bool use_speed_cost_;
  bool odom_updated_;
  bool local_map_updated_;
  bool scan_updated_;
  bool has_reached_;
  int velocity_samples_;
  int yawrate_samples_;
  int sim_time_samples_;
  int subscribe_count_th_;
  int odom_not_subscribe_count_;
  int local_map_not_subscribe_count_;
  int scan_not_subscribe_count_;

  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Publisher velocity_pub_;
  ros::Publisher candidate_trajectories_pub_;
  ros::Publisher selected_trajectory_pub_;
  ros::Publisher predict_footprints_pub_;
  ros::Publisher finish_flag_pub_;
  ros::Publisher path_cloud_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher new_agv_path_cloud_pub_;

  ros::Subscriber dist_to_goal_th_sub_;
  ros::Subscriber edge_on_global_path_sub_;
  ros::Subscriber footprint_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber local_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber target_velocity_sub_;
  ros::Subscriber pointcloud2_sub1_;
  ros::Subscriber pointcloud2_sub2_;
  ros::Subscriber pointcloud2_sub3_;
  ros::Subscriber pointcloud2_sub4_;
  ros::Subscriber control_data_sub_;

  geometry_msgs::Twist current_cmd_vel_;
  std::optional<geometry_msgs::PoseStamped> goal_msg_;
  geometry_msgs::PoseArray obs_list_;
  std::shared_ptr<std::vector<geometry_msgs::Point>> obstacles_points_;
  geometry_msgs::PolygonStamped footprint_;//footprint_可能存在，也可能不存在
  std::optional<nav_msgs::Path> edge_points_on_path_;

  std_msgs::Bool has_finished_;

  tf::TransformListener listener_;

  double sim_time_step;
  std::vector<std::pair<std::vector<State>, bool>> trajectories_results_;
  // std::vector<geometry_msgs::PolygonStampedPtr> footprints_;
  DWAPlanner::VehicleParams vehicle_;
  std::vector<State> local_path_;
  ros::Publisher pub_footprint_msg_;

  std::unordered_map<int, Node*> nodes_;
  std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open_;
  float local_path_length = 0.0;
  double wait_time_ = 1.0;
  bool is_car_stop_  = true;
};

#endif  // DWA_PLANNER_DWA_PLANNER_H
