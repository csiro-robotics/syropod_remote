#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

/**
 * This node is intended to convert joystic comands into  comands for hexapod robot.
 * this includes velocity vector and body pose vector. along with gait type and start stop commands for mapping and localisation and system
 */


// set up variables 


void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("I heard a joystic message");
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_remote");

  ros::NodeHandle n;
  //setup publish loop_rate (future perameter)
    ros::Rate loop_rate(100); 			//peramiterize  
  
  // subscribe to joystic topic
  ros::Subscriber sub = n.subscribe("joy", 1, JoyCallback);
 
  
  //setup publishers 
  //velocity publisher
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("desired_velocity",1);
  //pose publisher
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Twist>("desired_pose",1);
  //status publisher
  
  geometry_msgs::Twist vel;
  geometry_msgs::Twist pose; 
  
  
  
  while(ros::ok())
  {  //do maths
  
    
    //publish stuff
    velocity_pub.publish(vel);
    pose_pub.publish(pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
