#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left = img.width / 3;
    int right = 2 * img.width / 3;
    bool ball_found = false;
    int ball_pos = 0; // 0=left, 1=middle, 2=right

    // Loop through each pixel (step by 3 for RGB)
    for (int i = 0; i < img.height * img.step; i += 3)
    {
        int r = img.data[i];
        int g = img.data[i+1];
        int b = img.data[i+2];

        if (r == white_pixel && g == white_pixel && b == white_pixel)
        {
            int pixel_col = (i % img.step) / 3;
            if (pixel_col < left)
                ball_pos = 0;
            else if (pixel_col < right)
                ball_pos = 1;
            else
                ball_pos = 2;

            ball_found = true;
            break;
        }
    }

    // Drive based on ball position
    if (ball_found)
    {
        switch(ball_pos)
        {
            case 0: drive_robot(0.0, 0.5); break; // turn left
            case 1: drive_robot(0.5, 0.0); break; // go forward
            case 2: drive_robot(0.0, -0.5); break; // turn right
        }
    }
    else
    {
        drive_robot(0.0, 0.0); // stop if no ball
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

