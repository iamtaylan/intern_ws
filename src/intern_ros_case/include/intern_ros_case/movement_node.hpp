#ifndef MOVEMENT_NODE_HPP
#define MOVEMENT_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <map>
#include <string>

class MovementController {
public:
    MovementController();
    
    // Updates the robot's current position and orientation by processing incoming odometry data.
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // Ensures angular continuity by constraining the angle within the range of -PI to +PI.
    double normalize_angle(double angle);

    // Rotates the robot using angular error control until the target orientation is reached.
    void rotate_90_degrees();

    // Executes the sequence of tasks sequentially.
    void run_task();

private:
    ros::NodeHandle nh;
    ros::Publisher velocity_publisher;
    ros::Subscriber odom_subscriber;
    ros::Rate rate;

    // parameters
    double linear_speed;
    double angular_speed;
    double move_duration;

    // Variables
    double current_yaw;
    std::map<std::string, double> current_pose;
};

#endif // MOVEMENT_NODE_HPP