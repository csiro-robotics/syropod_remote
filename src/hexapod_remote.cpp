#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

/**
 * This node is intended to convert joystic comands into  comands for hexapod robot.
 * this includes velocity vector and body pose vector. along with gait type and start stop commands for mapping and localisation and system
 */
void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("I heard a joystic message");
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_remote");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joy", 1, JoyCallback);

  
  ros::spin();

  return 0;
}
