#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

/**
 * This node is intended to convert joystic comands into  comands for hexapod robot.
 * this includes velocity vector and body pose vector. along with gait type and start stop commands for mapping and localisation and system
 */


// set up variables 
geometry_msgs::Twist vel;
geometry_msgs::Twist pose; 

bool joy_one_flip; 

void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   
  
  if(joy->buttons[4]==1 && joy->buttons[5]==1)	//check if LB&RB are depressed 
  {
    ROS_INFO("Left and right buttons");
    pose.angular.x = joy->axes[0];
    pose.angular.y = joy->axes[1];
    pose.linear.z = joy->axes[4];
    pose.angular.z = joy->axes[3];
  }
  else if(joy->buttons[4]==1)			//if left button pressed 
  {
     ROS_INFO("Left button");
    vel.linear.x = joy->axes[0];
    vel.linear.y = joy->axes[1];
    vel.linear.z = joy->axes[4];
    vel.angular.z = joy->axes[3];
  }
  else if(joy->buttons[5]==1)			//if right button pressed 
  {
    ROS_INFO("right button");
    pose.linear.x = joy->axes[0];
    pose.linear.y = joy->axes[1];
    pose.linear.z = joy->axes[4];
    pose.angular.z = joy->axes[3];
  }
  else						//Proceed with normal controll
  {
    vel.linear.x = joy->axes[0];
    vel.linear.y = joy->axes[1];
    vel.angular.z = joy->axes[3];
    
  }
  

  //check button pressed
  
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

  // get / set peramiters
  
    if(joy_one_flip)
    {
      ROS_INFO("WOO");
    }
    
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
