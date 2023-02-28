#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#define LOOKAHEAD_DISTANCE 2.0 // meters
#define MAX_SPEED 1.0 // meters/second
#define PI 3.14159265358979323846

class DroneInterceptor
{
public:
    DroneInterceptor()
    {
        // Initialize ROS node and subscribers/publishers
        nh_ = ros::NodeHandle("~");
        sub_odom_ = nh_.subscribe("/drone/odom", 1, &DroneInterceptor::odomCallback, this);
        sub_target_ = nh_.subscribe("/drone/target", 1, &DroneInterceptor::targetCallback, this);
        pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/drone/cmd_vel", 1);

        // Initialize pure pursuit parameters
        lookahead_distance_ = LOOKAHEAD_DISTANCE;
        max_speed_ = MAX_SPEED;
        current_pos_ = geometry_msgs::Point();
        current_yaw_ = 0.0;
        last_target_pos_ = geometry_msgs::Point();
        cmd_vel_ = geometry_msgs::Twist();
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        // Update drone state
        current_pos_ = msg->pose.pose.position;
        current_yaw_ = getYaw(msg->pose.pose.orientation);

        // Calculate lookahead point
        if (last_target_pos_.x != 0.0 && last_target_pos_.y != 0.0 && last_target_pos_.z != 0.0)
        {
            double distance_to_target = getDistance(current_pos_, last_target_pos_);
            geometry_msgs::Point lookahead_point = getLookaheadPoint(distance_to_target);

            // Calculate command velocity
            double speed = max_speed_;
            if (distance_to_target < lookahead_distance_)
            {
                speed = max_speed_ * (distance_to_target / lookahead_distance_);
            }
            cmd_vel_.linear.x = speed * cos(current_yaw_);
            cmd_vel_.linear.y = speed * sin(current_yaw_);
            cmd_vel_.angular.z = getAngularVelocity(lookahead_point);
        }

        // Publish command velocity
        pub_cmd_.publish(cmd_vel_);
    }

    void targetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        // Update last target position
        last_target_pos_ = msg->pose.position;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_target_;
    ros::Publisher pub_cmd_;
    double lookahead_distance_;
    double max_speed_;
    geometry_msgs::Point current_pos_;
    double current_yaw_;
    geometry_msgs::Point last_target_pos_;
    geometry_msgs::Twist cmd_vel_;

    double getYaw(const geometry_msgs::Quaternion& quat)
    {
        // Convert quaternion to Euler angles and return yaw angle
        double roll, pitch, yaw;
        tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double getDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        // Calculate Euclidean distance between two points
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    }

    geometry_msgs::Point getLookaheadPoint(double distance_to_target)
    {
        // Calculate lookahead point based on distance to target
        geometry_msgs::Point target_pos = last_target_pos_;
        if (distance_to_target > lookahead_distance_)
        {
            double scale = lookahead_distance_ / distance_to_target;
            target_pos.x = current_pos_.x + scale * (last_target_pos_.x - current_pos_.x);
            target_pos.y = current_pos_.y + scale * (last_target_pos_.y - current_pos_.y);
            target_pos.z = current_pos_.z + scale * (last_target_pos_.z - current_pos_.z);
        }
        return target_pos;
    }

    double getAngularVelocity(const geometry_msgs::Point& lookahead_point)
    {
        // Calculate angular velocity using pure pursuit algorithm
        double heading_angle = atan2(lookahead_point.y - current_pos_.y, lookahead_point.x - current_pos_.x);
        double yaw_error = heading_angle - current_yaw_;
        while (yaw_error > PI)
        {
            yaw_error -= 2 * PI;
        }
        while (yaw_error < -PI)
        {
            yaw_error += 2 * PI;
        }
        return 2.0 * yaw_error / (lookahead_distance_ * lookahead_distance_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_interceptor");
    DroneInterceptor interceptor;
    ros::spin();
    return 0;
}