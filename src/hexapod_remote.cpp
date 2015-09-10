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

int axis_linear_x;
int axis_linear_y;
int axis_linear_z;
int axis_angular_z;
 
bool axis_linear_x_flip;
bool axis_linear_y_flip;
bool axis_linear_z_flip;
bool axis_angular_z_flip;

int pub_rate;

void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   
  
  if(joy->buttons[4]==1 && joy->buttons[5]==1)	//check if LB&RB are depressed 
  {
    ROS_INFO("Left and right buttons");
   if(axis_linear_x_flip == true)
    {
      pose.angular.x = joy->axes[axis_linear_x] * -1.0;
    }else
    {
      pose.angular.x = joy->axes[axis_linear_x];
    }  
     if(axis_linear_y_flip == true)
    {
      pose.angular.y = joy->axes[axis_linear_y] * -1.0;
    }else
    {
      pose.angular.y = joy->axes[axis_linear_y];
    }  
      if(axis_linear_z_flip == true)
    {
      pose.linear.z = joy->axes[axis_linear_z] * -1.0;
    }else
    {
      pose.linear.z = joy->axes[axis_linear_z];
    }  
     if(axis_angular_z_flip == true)
    {
      pose.angular.z = joy->axes[axis_angular_z] * -1.0;
    }else
    {
      pose.angular.z = joy->axes[axis_angular_z];
    }
    
  }
  else if(joy->buttons[4]==1)			//if left button pressed 
  {
    ROS_INFO("Left button");
     if(axis_linear_x_flip == true)
    {
      vel.linear.x = joy->axes[axis_linear_x] * -1.0;
    }else
    {
      vel.linear.x = joy->axes[axis_linear_x];
    }  
     if(axis_linear_y_flip == true)
    {
      vel.linear.y = joy->axes[axis_linear_y] * -1.0;
    }else
    {
      vel.linear.y = joy->axes[axis_linear_y];
    }  
      if(axis_linear_z_flip == true)
    {
      vel.linear.z = joy->axes[axis_linear_z] * -1.0;
    }else
    {
      vel.linear.z = joy->axes[axis_linear_z];
    }  
     
     if(axis_angular_z_flip == true)
    {
      vel.angular.z = joy->axes[axis_angular_z] * -1.0;
    }else
    {
      vel.angular.z = joy->axes[axis_angular_z];
    }  
    
  }
  else if(joy->buttons[5]==1)			//if right button pressed 
  {
    ROS_INFO("right button");
    if(axis_linear_x_flip == true)
    {
      pose.linear.x = joy->axes[axis_linear_x] * -1.0;
    }else
    {
      pose.linear.x = joy->axes[axis_linear_x];
    }  
     if(axis_linear_y_flip == true)
    {
      pose.linear.y = joy->axes[axis_linear_y] * -1.0;
    }else
    {
      pose.linear.y = joy->axes[axis_linear_y];
    }  
      if(axis_linear_z_flip == true)
    {
      pose.linear.z = joy->axes[axis_linear_z] * -1.0;
    }else
    {
      pose.linear.z = joy->axes[axis_linear_z];
    }  
     if(axis_angular_z_flip == true)
    {
      pose.angular.z = joy->axes[axis_angular_z] * -1.0;
    }else
    {
      pose.angular.z = joy->axes[axis_angular_z];
    }  
    
  }
  else						//Proceed with normal controll
  {
     if(axis_linear_x_flip == true)
    {
      vel.linear.x = joy->axes[axis_linear_x] * -1.0;
    }else
    {
      vel.linear.x = joy->axes[axis_linear_x];
    }  
     if(axis_linear_y_flip == true)
    {
      vel.linear.y = joy->axes[axis_linear_y] * -1.0;
    }else
    {
      vel.linear.y = joy->axes[axis_linear_y];
    }  
     if(axis_angular_z_flip == true)
    {
      vel.angular.z = joy->axes[axis_angular_z] * -1.0;
    }else
    {
      vel.angular.z = joy->axes[axis_angular_z];
    }  
    
  }
  

  //check button pressed
  
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_remote");

  ros::NodeHandle n;

  // get and set peramiter defaults if perameters arn't on the param server  
  n.param("hexapod_remote/axis_linear_x", axis_linear_x, 0);
  n.param("hexapod_remote/axis_linear_y", axis_linear_y, 1);
  n.param("hexapod_remote/axis_linear_z", axis_linear_z, 4);
  n.param("hexapod_remote/axis_angular_z", axis_angular_z, 3);
  n.param("hexapod_remote/axis_linear_x_flip", axis_linear_x_flip, false);
  n.param("hexapod_remote/axis_linear_y_flip", axis_linear_y_flip, false);
  n.param("hexapod_remote/axis_linear_z_flip", axis_linear_z_flip, false);
  n.param("hexapod_remote/axis_angular_z_flip", axis_angular_z_flip, false);
  
  n.param("hexapod_remote/pub_rate",pub_rate, 50);
  

  //setup publish loop_rate 
   ros::Rate loop_rate(pub_rate); 			//peramiterize  
  
  // subscribe to joystic topic
  ros::Subscriber sub = n.subscribe("joy", 1, JoyCallback);
 
  
  //setup publishers 
  //velocity publisher
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("desired_velocity",1);
  //pose publisher
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Twist>("desired_pose",1);
  //status publisher


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
