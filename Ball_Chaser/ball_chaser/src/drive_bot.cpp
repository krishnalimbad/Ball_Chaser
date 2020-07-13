#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool drive_request_callback(ball_chaser::DriveToTarget::Request& reqe,
                            ball_chaser::DriveToTarget::Response& resp)
{
  ROS_INFO("DriveToTargetRequest received - linear_x: %1.2f, angular_z: %1.2f", (float)reqe.linear_x, (float)reqe.angular_z);

  
  geometry_msgs::Twist cmd_vel_;

  cmd_vel_.linear.x = reqe.linear_x;
  cmd_vel_.angular.z = reqe.angular_z; 
  
  motor_command_publisher.publish(cmd_vel_);

  // Return a response message
  resp.msg_feedback = "Velocity cmd sent: linear_x = " + std::to_string(cmd_vel_.linear.x) + ", angular_z = " + std::to_string(cmd_vel_.angular.z);

  ROS_INFO_STREAM(resp.msg_feedback);
  

  return true;

}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", drive_request_callback);
    ROS_INFO("Ready to send command velocities");

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
   

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
