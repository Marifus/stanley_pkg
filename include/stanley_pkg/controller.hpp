#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

namespace stanley
{

    class Stanley
    {
        bool input_log, output_log;

        ros::NodeHandle& nh_;
        ros::Subscriber path_sub;
        ros::Subscriber odom_sub;
        ros::Publisher path_pub;
        ros::Publisher ctrl_pub;

        nav_msgs::Path path;
        nav_msgs::Odometry vehicle_odom;

        double cte_coefficient;
        double velocity_coefficient;
        double axle_length;

        double velocity;

        bool ReadParameters();
/*         void PathCallback(const nav_msgs::Odometry::ConstPtr& msg); */
        void PathCallback(const nav_msgs::Path::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void ControlOutput();
        double StanleyAlgorithm(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, double current_velocity, double Kcte, double Kv);
        int ClosestWaypointIndex(const geometry_msgs::Pose& current_pose, const nav_msgs::Path& t_path);
        geometry_msgs::Pose LocalTransform(const geometry_msgs::Pose& origin_pose, const geometry_msgs::Pose& target_point_pose);
        double GetYaw(const geometry_msgs::Quaternion& q);
        double CalculatePathYaw(const nav_msgs::Path& t_path, int closest_idx);

        public:

            Stanley(ros::NodeHandle& nh);

    };


}