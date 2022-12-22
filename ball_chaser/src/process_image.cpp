#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Map value from one range onto another
float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Moving the Robot");

    // Request updated linear and angular velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the drive_robot service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Initialize variables
    int white_pixel = 255, 
        count_white_pixels = 0;
    float average_step = 0, 
        sum_step = 0, 
        max_linear_velocity = 0.5, 
        max_angular_velocity = 1.0, 
        angle = 0, 
        speed = 0;
    bool robot_moving = false;

    // Loop through all pixels in the image and record their instances and horizontal locations
    for (int i = 0; i < img.height * img.step; i+=3) 
    {
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) 
        {
            count_white_pixels += 1;
            sum_step += i % img.step;
        }
    }

    // Determine the command to send to the robot via the drive_robot service
    if (count_white_pixels == 0 && robot_moving)
    {
        // Request a stop
        drive_robot(0, 0);
        robot_moving = false;
    }
    else if (count_white_pixels != 0)
    {
        // Drive robot toward the white ball

        // Map step to percent of max angular velocity
        average_step = sum_step / count_white_pixels;

        angle = map_value(average_step, 0, img.step, 1.0, -1.0) * max_angular_velocity;
        ROS_INFO_STREAM("Average Step Calc = " + std::to_string(average_step));

        // Calculate percent of max linear velocity based on size of the ball in the image
        speed = (1.0 - (count_white_pixels*2/(img.height*img.step/3))) * max_linear_velocity;
        ROS_INFO_STREAM("Angle Calc = " + std::to_string(angle) + "    Speed Calc = " + std::to_string(speed));

        // Request a robot move
        drive_robot(speed, angle);

        robot_moving = true;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}