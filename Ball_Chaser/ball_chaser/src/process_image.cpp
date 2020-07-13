#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient Sclient;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  
  // Call the drive_bot service and pass the requested velocities
  if (!Sclient.call(srv)) {
    ROS_ERROR("failed to acll service");
  }
    // TODO: Request a service and pass the velocities to it to drive the robot
}


void process_image_callback(const sensor_msgs::Image img)
{
  int pixel_h = -1, pixel_s = -1; 

  for (int i = 0; i < img.height * img.step; i++) {
    if ((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255)) {
      pixel_h = i / img.step;
      pixel_s = i % img.step;
      break;
    } 
  }
  
  float lin_x = 0.0, ang_z = 0.0;
  if (pixel_s <= img.step * 0.4 && pixel_s >= 0) { // Left 
    ang_z = 0.5;
  } else if (pixel_s > img.step *0.7  && pixel_s <= img.step) { // Right 
    ang_z = -0.5;
  } else if (pixel_s != -1) { // Forwared 
    lin_x = 0.5;
    ROS_INFO_STREAM("target detected... driving .");
  }
          
  // call drive_bot function and pass velocities
  drive_robot(lin_x, ang_z); // drive the bot
  
  // Publish  info
  if (pixel_s != -1) { // Forwared 
    ROS_INFO_STREAM("target detected... driving .");
  }
  else{
    ROS_INFO_STREAM("target not detected... Stop .");
  }

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    Sclient = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

