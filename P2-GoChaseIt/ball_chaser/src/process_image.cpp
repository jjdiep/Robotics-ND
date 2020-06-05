#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int ball_dir;
    bool ball_found = false;
    int white_pixel[] = {255,255,255};
    float left_region_max = (float) img.step/3;
    float mid_region_max = (float) 2*img.step/3;
    const int num_channels = 3;
    // printf("left region max %f\n", left_region_max);
    // printf("mid region max %f\n", mid_region_max);

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for (int i = 0; i < img.height * img.step; i = i + 3) {
        int img_pixel[] = {img.data[i],img.data[i+1],img.data[i+2]};
        if (img_pixel[0] == white_pixel[0] && img_pixel[1] == white_pixel[1] && img_pixel[2] == white_pixel[2]) {
            ball_dir = i % img.step;
            if (ball_dir > left_region_max && ball_dir < mid_region_max) { // mid region
                drive_robot(.2,0);
            } 
            else if (ball_dir > mid_region_max) { // right region
                drive_robot(.2,-1.507);
            }
            else { // left region
                drive_robot(.2,1.507);
            }
            ball_found = true;
            break;
        }
    }
    if (!ball_found) // Default: does not see ball, stop
        drive_robot(0,0);

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