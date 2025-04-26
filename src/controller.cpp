#include "stanley_pkg/controller.hpp"

namespace stanley
{

    Stanley::Stanley(ros::NodeHandle& nh) : nh_(nh)
    {
        if (!ReadParameters())
        {
            ROS_ERROR("Parametreler Okunamadi.");
            ros::requestShutdown();
        }

        path_sub = nh_.subscribe("/shortest_path", 10, &Stanley::PathCallback, this);
        odom_sub = nh_.subscribe("/odom", 10, &Stanley::OdomCallback, this);
        path_pub = nh_.advertise<nav_msgs::Path>("/path", 10);
        ctrl_pub = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
    }

    bool Stanley::ReadParameters()
    {
        if (!nh_.getParam("cte_coefficient", cte_coefficient)) return false;
        if (!nh_.getParam("velocity_coefficient", velocity_coefficient)) return false;
        if (!nh_.getParam("axle_length", axle_length)) return false;

        ROS_INFO("CTE Katsayisi: %f", cte_coefficient);
        ROS_INFO("Hiz Katsayisi: %f", velocity_coefficient);
        ROS_INFO("Sase Uzunlugu: %f", axle_length);

        return true;
    }

    void Stanley::PathCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = msg->pose.pose;

        path.poses.push_back(pose_stamped);
        path.header = msg->header;
        path_pub.publish(path);
    }

/* 
    void Stanley::PathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path = *msg;
    }
 */

    void Stanley::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        vehicle_odom = *msg;

        double current_heading = GetYaw(vehicle_odom.pose.pose.orientation);

        vehicle_odom.pose.pose.position.x += cos(current_heading)*axle_length;
        vehicle_odom.pose.pose.position.y += sin(current_heading)*axle_length;

        ROS_INFO("Mevcut Konum: [%f, %f]", vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y);
        ROS_INFO("Mevcut Yaw: [%f]", current_heading);

        if (path.poses.size() != 0) ControlOutput();
    }

    void Stanley::ControlOutput()
    {
        double steering_angle = StanleyAlgorithm(vehicle_odom.pose.pose, path, velocity, cte_coefficient, velocity_coefficient);
        double steering_angle_degree = steering_angle * (180 / M_PI);

        if (steering_angle_degree > 100) steering_angle = 100 * (M_PI / 180);
        else if (steering_angle_degree < -100) steering_angle = -100 * (M_PI / 180);
        
        ROS_INFO("Donus Acisi: %f", steering_angle);

        autoware_msgs::VehicleCmd ctrl_msg;
        ctrl_msg.header.stamp = ros::Time::now();
        ctrl_msg.twist_cmd.twist.linear.x = velocity;
        ctrl_msg.twist_cmd.twist.angular.z = steering_angle;
        ctrl_pub.publish(ctrl_msg);
    }

    double Stanley::StanleyAlgorithm(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path, double current_velocity, double Kcte, double Kv)
    {
        int closest_idx = ClosestWaypointIndex(current_point_pose, t_path); 
        geometry_msgs::Pose closest_point = t_path.poses[closest_idx].pose;
        double path_yaw = closest_idx+1 < t_path.poses.size() ? std::atan2(t_path.poses[closest_idx+1].pose.position.y - t_path.poses[closest_idx].pose.position.y, t_path.poses[closest_idx+1].pose.position.x - t_path.poses[closest_idx].pose.position.x) : atan2(t_path.poses[closest_idx].pose.position.y - t_path.poses[closest_idx-1].pose.position.y, t_path.poses[closest_idx].pose.position.x - t_path.poses[closest_idx-1].pose.position.x);
        double current_yaw = GetYaw(current_point_pose.orientation);

        double heading_error = path_yaw - current_yaw;

        double transformed_closest_point[3] = {0, 0, 0};
        LocalTransform(current_point_pose, closest_point, transformed_closest_point);

/*         double yaw_to_path = atan2(transformed_closest_point[1], transformed_closest_point[0]);
        double cte = std::cos(yaw_to_path) * transformed_closest_point[0] - std::sin(yaw_to_path) * transformed_closest_point[1]; */

        double cte = std::sqrt(std::pow(transformed_closest_point[0], 2) + std::pow(transformed_closest_point[1], 2));
        if (transformed_closest_point[1] < 0) cte = -cte;

        double steering = heading_error + std::atan2((cte * Kcte),(Kv * current_velocity));

        while (steering>M_PI) steering -= 2*M_PI;
        while (steering<-M_PI) steering += 2*M_PI;

        return steering;
    }

    int Stanley::ClosestWaypointIndex(const geometry_msgs::Pose& current_point_pose, const nav_msgs::Path& t_path)
    {
        int closest_point_index;
        double min_distance2;

        for (int i = 0; i < path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped& waypoint = path.poses[i];

            if (i == 0)
            {
                closest_point_index = i;
                min_distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);
            }

            else 
            {
                double distance2 = std::pow((waypoint.pose.position.x - current_point_pose.position.x), 2) + std::pow((waypoint.pose.position.y - current_point_pose.position.y), 2);

                if (distance2 < min_distance2)
                {
                    closest_point_index = i;
                    min_distance2 = distance2;
                }
            }
        }

        return closest_point_index;
    }

    void Stanley::LocalTransform(const geometry_msgs::Pose& current_point_pose, const geometry_msgs::Pose& target_point_pose, double transformed_vector[3])
    {
        
        double tx = -1 * current_point_pose.position.x;
        double ty = -1 * current_point_pose.position.y;

        double current_heading_ = GetYaw(current_point_pose.orientation);

        double target_vec[3] = {target_point_pose.position.x, target_point_pose.position.y, 1};

        double TransformationMatrix[3][3] = {
            {cos(-current_heading_), -sin(-current_heading_), cos(-current_heading_) * tx - sin(-current_heading_) * ty},
            {sin(-current_heading_), cos(-current_heading_), sin(-current_heading_) * tx + cos(-current_heading_) * ty},
            {0.0, 0.0, 1.0}
        };

        for (int i=0; i<3; i++) {
        
            transformed_vector[i] = 0;

            for (int j=0; j<3; j++) {

                transformed_vector[i] += TransformationMatrix[i][j] * target_vec[j];

           }
        }

    }

    double Stanley::GetYaw(const geometry_msgs::Quaternion& q)
    {
        return atan2(2.0 * (q.w * q.z + q.x * q.y) , 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

}