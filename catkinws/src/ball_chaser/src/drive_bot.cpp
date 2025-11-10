#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h" // Include the header

// ROS publisher for motor commands
ros::Publisher motor_command_publisher;

// This function executes whenever a drive_bot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest received - linear_x: %.2f, angular_z: %.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish requested wheel velocities
    motor_command_publisher.publish(motor_command);

    // Return a feedback message
    res.msg_feedback = "Wheel velocities set - linear_x: " + std::to_string(req.linear_x) +
                       ", angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize the drive_bot node
    ros::init(argc, argv, "drive_bot");

    // Create a NodeHandle object
    ros::NodeHandle n;

    // Define a publisher to publish motor commands to /cmd_vel
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with the handle_drive_request callback
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send motor commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}

