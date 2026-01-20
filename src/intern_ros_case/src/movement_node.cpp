#include "intern_ros_case/movement_node.hpp"

MovementController::MovementController() : rate(20) {
    // parameters
    ros::NodeHandle pnh("~");
    pnh.param<double>("linear_speed", linear_speed, 0.2);
    pnh.param<double>("angular_speed", angular_speed, 0.3);
    pnh.param<double>("move_duration", move_duration, 5.0);

    // Variables
    current_yaw = 0.0;
    current_pose["x"] = 0.0;
    current_pose["y"] = 0.0;

    // Publisher/Subscriber
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_subscriber = nh.subscribe("/odom", 10, &MovementController::odom_callback, this);
}

void MovementController::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose["x"] = msg->pose.pose.position.x;
    current_pose["y"] = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    current_yaw = yaw;

    // Output:
    ROS_INFO("X: %.2f | Y: %.2f | Yaw: %.2f rad", 
             current_pose["x"], current_pose["y"], current_yaw);
}

double MovementController::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void MovementController::rotate_90_degrees() {
    geometry_msgs::Twist vel_msg;
    double target_rad = 1.57;   // 90-Degree
    
    double start_yaw = current_yaw;
    double target_yaw = normalize_angle(start_yaw + target_rad);
    
    ROS_INFO("Starting Rotation. Target Yaw: %.2f", target_yaw);

    while (ros::ok()) {
        ros::spinOnce(); 

        double angle_error = normalize_angle(target_yaw - current_yaw);
        
        if (std::abs(angle_error) < 0.01) {
            break;
        }
        
        vel_msg.angular.z = (angle_error > 0) ? angular_speed : -angular_speed;
        velocity_publisher.publish(vel_msg);
        rate.sleep();
    }

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    ROS_INFO("Rotation completed.");
}

void MovementController::run_task() {
    // 1. Forward Motion
    ROS_INFO("Stage 1: Moving forward...");
    ros::Time start_time = ros::Time::now();
    geometry_msgs::Twist vel_msg;
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < move_duration) {
        vel_msg.linear.x = linear_speed;
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // 2. Stop
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    ros::Duration(1.0).sleep();

    // 3. 90-Degree Rotation 
    ROS_INFO("Stage 2: Rotating 90 degrees...");
    rotate_90_degrees();

    // 4. Stop
    ROS_INFO("All tasks completed successfully.");
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "movement_node");
    
    try {
        MovementController controller;
        ros::Duration(1.0).sleep(); 
        controller.run_task();
    }
    catch (ros::Exception& e) {
        ROS_ERROR("ROS Error: %s", e.what());
    }

    return 0;
}