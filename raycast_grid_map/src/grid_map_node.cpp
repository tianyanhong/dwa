#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include "raycast_grid_map/grid_map_config.h"
#include "raycast_grid_map/grid_map.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "raycast_grid_map/astar_search.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

    footprint_padding_ = 0.01;
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>( "/no_ground_pc1", 1 , &GridMapGenerator::pointCloudCallback, this);
    path_sub_ = nh_.subscribe("/agv_path",1,&GridMapGenerator::pathCallback,this);
    sub_footprint_ = nh_.subscribe<geometry_msgs::PolygonStamped>("/footprint", 1, &GridMapGenerator::footprintCb, this);
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
    astar_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/astar_path_cloud", 1);
    pub_footprint_msg_ =
        nh_.advertise<geometry_msgs::PolygonStamped>("/footprint_2", 1);
    new_agv_path_cloud_pub_ = nh_.advertise<nav_msgs::Path>("/new_agv_path", 10);
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
        if(latest_path_.poses.empty())
        {
            ROS_INFO("no pointcloud or path points");
            return;
        }
        buildMap();
        publish();
        goal.x() = latest_path_.poses.back().pose.position.x;
        goal.y() = latest_path_.poses.back().pose.position.y;
        geometry_msgs::Quaternion q = latest_path_.poses.back().pose.orientation;

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        goal.z() = yaw;
        //路径穿过膨胀的点,start_p,end_p;
        Eigen::Vector3d start_p,end_p;
        
        if(getAstar_start_and_end_points(start_p,end_p))
        {
            std::vector<Eigen::Vector3d> astar_search_path_ps = AStarSearch(*grid_map_,start_p,end_p);
            ROS_INFO("get a star path points %ld",astar_search_path_ps.size());
            if(astar_search_path_ps.empty())
            {
                ROS_INFO("get a star path points is 0");
                return;
            }
            
            //计算astar_search_path_ps的角度
            calcu_astar_angle(astar_search_path_ps);
            //筛选满足条件的astar_pc
            std::vector<Eigen::Vector3d> pruned_path;
            for(auto& p:astar_search_path_ps)
            {
                // ROS_INFO("isFootprintColliding(p,*grid_map_) %d",isFootprintColliding(p,*grid_map_));
                if (!isFootprintColliding(p,*grid_map_))
                {
                    pruned_path.push_back(p);
                } 
            }
            ROS_INFO("pruned_path size %ld",pruned_path.size());

            if(pruned_path.empty())
            {
                ROS_INFO("pruned_path size is empty()");
                return;
            }

            Eigen::Vector3d start(0,0,0);
            //连接这些点
            std::vector<Eigen::Vector3d> results;
            interpolate(start,pruned_path.front(),results);
            for(auto& p:pruned_path)
            {
                results.push_back(p);
            }
            interpolate(pruned_path.back(),goal,results);
            publishAstarPathCloud(results,astar_pc_pub_);

            for(auto& p:results)
            {
                // ROS_INFO("isFootprintColliding(p,*grid_map_) %d",isFootprintColliding(p,*grid_map_));
                if (!isFootprintColliding(p,*grid_map_))
                {
                    ROS_INFO("The path has collided");
                    return;
                } 
            }

            //转到map坐标系下
            nav_msgs::Path new_path;
            transfromToMap(results,new_path);
            nav_msgs::Path new_path_out;
            interpolatePathUniform(new_path,new_path_out,0.01);
            ROS_INFO("new_path_out size = %ld",new_path_out.poses.size());
            new_agv_path_cloud_pub_.publish(new_path_out);      

        }else{
            //维持原来的路径发布gav_path

        
        }
        get_new_sensors_data_ =  false;


        

    }
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if(!get_new_sensors_data_)
        {
            get_new_sensors_data_ = true;
        }
        latest_cloud_ = *msg;
        has_cloud_ = true;
    }
    void pathCallback(const nav_msgs::PathConstPtr& msg)
    {
        if (msg->poses.empty())
            return;
        nav_msgs::Path edge_points_on_path_ = *msg;
        edge_points_on_path_.header.frame_id = "map";
        for (auto &pose : edge_points_on_path_.poses)
        {
            //local_path 本来就是base_link下的
            pose.header.frame_id = "map";
            pose.pose.position.x = pose.pose.position.x*0.001;
            pose.pose.position.y = pose.pose.position.y*0.001;
        }

        nav_msgs::Path transformed_path;
        transformed_path.header.stamp = ros::Time::now();
        transformed_path.header.frame_id = "base_link";

        try
        {
            // 查找 map -> base_link 的变换
            // geometry_msgs::TransformStamped map_to_base =
            //     tf_buffer_.lookupTransform(
            //         "base_link",
            //         msg->header.frame_id,   // 通常是 "map" 或 "odom"
            //         ros::Time(0)
            //     );

            for (const auto& pose_stamped : edge_points_on_path_.poses)
            {
                geometry_msgs::PoseStamped pose_in_base;
                // tf2::doTransform(pose_stamped, pose_in_base, map_to_base);
                listener_.transformPose("base_link", ros::Time(0), pose_stamped, pose_stamped.header.frame_id, pose_in_base);
                transformed_path.poses.push_back(pose_in_base);
            }

            latest_path_ = transformed_path;
            ROS_INFO("latest path point %ld",latest_path_.poses.size());
            has_path_ = true;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("Path transform failed: %s", ex.what());
        }
    }
    void footprintCb(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        footprint_ = *msg;
        ROS_INFO("footprint size = %lu", msg->polygon.points.size());

        for (auto &point : footprint_.polygon.points)
        {
          point.x += point.x < 0 ? -footprint_padding_ : footprint_padding_;
          point.y += point.y < 0 ? -footprint_padding_ : footprint_padding_;
        }
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
            addCollision(pos, 0.45);
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
    bool getAstar_start_and_end_points(Eigen::Vector3d &start_p,Eigen::Vector3d &end_p)
    {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();

        bool found_start = false;
        bool found_end   = false;
        for(const auto& pose:latest_path_.poses)
        {
            Eigen::Vector3d p;
            p.x() = pose.pose.position.x;
            p.y() = pose.pose.position.y;
            p.z() = 0;
            if(!grid_map_->isInObstacleOrInflated(p))
            {
                continue;
            }
            double x = p.x();

            // 最近点（start）
            if (x < min_x)
            {
                min_x = x;
                start_p = p;
                found_start = true;
            }

            // 最远点（end）
            if (x > max_x)
            {
                max_x = x;
                end_p = p;
                found_end = true;
            }

            if (!found_start || !found_end)
            {
                ROS_WARN("No valid A* start/end points found in obstacle area!");
                return false;
            }

        }
        return true;
    
    }
    void publishAstarPathCloud(
        const std::vector<Eigen::Vector3d>& path,
        const ros::Publisher& pub,
        const std::string& frame_id = "base_link")
    {
        if (path.empty())
            return;

        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = frame_id;
        cloud.height = 1;
        cloud.width = path.size();
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        // 定义字段 x, y, z
        cloud.fields.resize(3);
        cloud.fields[0].name = "x";
        cloud.fields[1].name = "y";
        cloud.fields[2].name = "z";

        int offset = 0;
        for (auto& field : cloud.fields)
        {
            field.offset = offset;
            field.datatype = sensor_msgs::PointField::FLOAT32;
            field.count = 1;
            offset += sizeof(float);
        }

        cloud.point_step = offset;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cloud.row_step);

        // 写入数据
        float* data_ptr = reinterpret_cast<float*>(cloud.data.data());
        for (size_t i = 0; i < path.size(); ++i)
        {
            data_ptr[i * 3 + 0] = path[i].x();
            data_ptr[i * 3 + 1] = path[i].y();
            data_ptr[i * 3 + 2] = path[i].z();
        }

        pub.publish(cloud);
    }
    void calcu_astar_angle(std::vector<Eigen::Vector3d>& path)
    {
        for (size_t i = 1; i < path.size(); ++i)
        {
            double dx = path[i].x() - path[i - 1].x();
            double dy = path[i].y() - path[i - 1].y();
            path[i].z() = std::atan2(dy, dx);
        }
    }
    bool isFootprintColliding(
        const Eigen::Vector3d& pose,
        const GridMap& map)
    {
        const geometry_msgs::PolygonStamped footprint = move_footprint(pose);
        for(int index = 0; index < map.obstacle.size(); index++)
        {
            if(map.obstacle[index] == 0)
            {
                continue;
            }
            double xw = 0,yw = 0;
            if(indexToWorld(map,index,xw,yw))
            {
                // ROS_INFO("xw = %lf,yw = %lf",xw,yw);
                geometry_msgs::Point obstacle;
                obstacle.x = xw;
                obstacle.y = yw;

                if (is_inside_of_robot(obstacle, footprint))
                {
                    return true; 
                }
            }
        }
        return false;
    }
    geometry_msgs::PolygonStamped move_footprint(const Eigen::Vector3d &target_pose)
    {
        geometry_msgs::PolygonStamped footprint;
        // if (use_footprint_)
        // {
            footprint = footprint_;
        // }
        footprint.header.stamp = ros::Time::now();

        for (auto &point : footprint.polygon.points)
        {
            Eigen::VectorXf point_in(2);
            point_in << point.x, point.y;
            Eigen::Matrix2f rot;
            rot = Eigen::Rotation2Df(target_pose.z());
            const Eigen::VectorXf point_out = rot * point_in;

            point.x = point_out.x() + target_pose.x();
            point.y = point_out.y() + target_pose.y();
        }
        pub_footprint_msg_.publish(footprint);
        return footprint;
    }

    bool is_inside_of_robot(const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped footprint)
    {
        if (footprint.polygon.points.empty())
            return false;

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

    bool indexToWorld(
        const GridMap& map,
        int index,
        double& wx,
        double& wy)
    {
        if (index < 0 || index >= map.width * map.height)
            return false;

        int x = index % map.width;
        int y = index / map.width;

        wx = map.x_min + (x + 0.5) * map.resolution;
        wy = map.y_min + (y + 0.5) * map.resolution;
        return true;
    }

    void interpolate(Eigen::Vector3d &start_p,Eigen::Vector3d &end_p,std::vector<Eigen::Vector3d> &results)
    {
        const double step = 0.10; // 10cm
        std::vector<Eigen::Vector3d> result;   
        double dx = end_p.x() - start_p.x();
        double dy = end_p.y() - start_p.y();
        double length = std::hypot(dx, dy);
        // 防止除零
        if (length < 1e-6)
        {
            results.push_back(start_p);
            return;
        }
        int num_points = static_cast<int>(length / step);

        // 四元数表示 yaw
        tf2::Quaternion q_start, q_end;
        q_start.setRPY(0.0, 0.0, start_p.z());
        q_end.setRPY(0.0, 0.0, end_p.z());

        for (int i = 0; i <= num_points; ++i)
        {
            double t = (i * step) / length;

            // 位置线性插值
            Eigen::Vector3d p;
            p.x() = start_p.x() + t * dx;
            p.y() = start_p.y() + t * dy;

            // 姿态球面插值
            tf2::Quaternion q = q_start.slerp(q_end, t);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            p.z() = yaw;
            results.push_back(p);
        }
        // 确保终点一定存在
        results.push_back(end_p);
    }
    void transfromToMap(std::vector<Eigen::Vector3d> &results,nav_msgs::Path &new_path)
    {
        for(auto& p:results)
        {
            geometry_msgs::PoseStamped pose;

            pose.header.frame_id = "base_link";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = p.x();//1000
            pose.pose.position.y = p.y();//1000  //发布一个新的new_agv_path;
            // pose.pose.position.z = 0.0;

            // yaw 转四元数（单位：弧度）
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, p.z()); 
            pose.pose.orientation = tf2::toMsg(q);
            geometry_msgs::PoseStamped pose_in_map;
            try{
                listener_.transformPose("map", ros::Time(0), pose, pose.header.frame_id, pose_in_map); 
            }catch (const tf2::TransformException& ex)
            {
                ROS_WARN("TF transform failed: %s", ex.what());
                continue;
            }

            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            new_path.poses.push_back(pose_in_map);
        }


    }

    void interpolatePathUniform(
        const nav_msgs::Path& in,
        nav_msgs::Path& out,
        double step = 0.01)
    {
        out.header = in.header;
        out.poses.clear();

        for (size_t i = 1; i < in.poses.size(); ++i)
        {
            const auto& pose0 = in.poses[i - 1].pose;
            const auto& pose1 = in.poses[i].pose;

            Eigen::Vector3d p0(
                pose0.position.x,
                pose0.position.y,
                pose0.position.z);

            Eigen::Vector3d p1(
                pose1.position.x,
                pose1.position.y,
                pose1.position.z);

            tf2::Quaternion q0, q1;
            tf2::fromMsg(pose0.orientation, q0);
            tf2::fromMsg(pose1.orientation, q1);

            double dist = (p1 - p0).norm();
            int n = std::max(1, static_cast<int>(dist / step));

            for (int k = 0; k <= n; ++k)
            {
                double t = static_cast<double>(k) / n;

                geometry_msgs::PoseStamped ps;
                ps.header = in.poses[i - 1].header;

                // 位置
                Eigen::Vector3d p = p0 + t * (p1 - p0);
                ps.pose.position.x = p.x();
                ps.pose.position.y = p.y();
                ps.pose.position.z = p.z();

                // 姿态（✅ Slerp）
                tf2::Quaternion q = q0.slerp(q1, t);
                ps.pose.orientation = tf2::toMsg(q);

                out.poses.push_back(ps);
            }
        }
    }


private:
    ros::NodeHandle nh_;

    ros::Publisher pub_;
    ros::Publisher astar_pc_pub_;
    ros::Subscriber sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber sub_footprint_;
    ros::Publisher new_agv_path_cloud_pub_;

    ros::Timer timer_;

    nav_msgs::Path latest_path_;
    bool has_path_ = false;
    sensor_msgs::PointCloud2 latest_cloud_;
    bool has_cloud_ = false;

    GridMapConfig config_;
    std::unique_ptr<GridMap> grid_map_;
    tf2_ros::Buffer tf_buffer_;
    tf::TransformListener listener_;
    bool get_new_sensors_data_=false;
    geometry_msgs::PolygonStamped footprint_;
    float footprint_padding_;
    ros::Publisher pub_footprint_msg_;
    Eigen::Vector3d  goal;
    

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map_generator");
    GridMapGenerator gen;
    ros::spin();
    return 0;
}