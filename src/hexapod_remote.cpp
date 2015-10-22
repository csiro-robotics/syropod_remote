#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

/**
 * This node is intended to convert joystic comands into  comands for hexapod robot.
 * this includes velocity vector and body pose vector. along with gait type and start stop commands for mapping and localisation and system
 */


// set up variables 
geometry_msgs::Twist vel;
geometry_msgs::Twist pose; 
std_msgs::Bool start_state;

int axis_linear_x;
int axis_linear_y;
int axis_linear_z;
int axis_angular_x;
int axis_angular_y;
int axis_angular_z;

int A_button;
int B_button;
int X_button;
int Y_button;
int Left_button;
int Right_button;
int Back_button;
int Start_button;
int Logitech_button;
int Left_joy_button;
int Right_joy_button;

bool axis_linear_x_flip;
bool axis_linear_y_flip;
bool axis_linear_z_flip;
bool axis_angular_x_flip;
bool axis_angular_y_flip;
bool axis_angular_z_flip;

int pub_rate;

void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //CHECK FOR START BUTTON PRESS
  if (joy->buttons[Start_button] == 1) //Start button
  {
    start_state.data = true;
  }
  
  if (joy->buttons[Back_button] == 1) //Back button
  {
    start_state.data = false;
  }	

 // CHECK CONTROL MODE   
  if(joy->buttons[Left_button]==1 && joy->buttons[Right_button]==1)	//check if LB&RB are depressed 
  {
    ROS_INFO("Left and right buttons");

    vel.linear.x = joy->axes[axis_linear_x];
    vel.linear.y = joy->axes[axis_linear_y];
    vel.linear.z = joy->axes[axis_linear_z];
    pose.angular.z = joy->axes[axis_angular_x];
    pose.linear.z = joy->axes[axis_angular_y];
    vel.angular.z = joy->axes[axis_angular_z];
    
    if (axis_linear_x_flip) vel.linear.x *= -1.0;
    if (axis_linear_y_flip) vel.linear.y *= -1.0;
    if (axis_linear_z_flip) vel.linear.z *= -1.0;
    if (axis_angular_x_flip) pose.angular.x *= -1.0;
    if (axis_angular_y_flip) pose.angular.y *= -1.0;
    if (axis_angular_z_flip) vel.angular.z *= -1.0;
  }
  else if(joy->buttons[Left_button]==1)			//if left button pressed 
  {
    ROS_INFO("Left button");

    vel.linear.x = joy->axes[axis_linear_x];
    vel.linear.y = joy->axes[axis_linear_y];
    vel.linear.z = joy->axes[axis_linear_z];
    pose.linear.x = joy->axes[axis_angular_x];
    pose.linear.y = joy->axes[axis_angular_y];
    vel.angular.z = joy->axes[axis_angular_z];

    if (axis_linear_x_flip) vel.linear.x *= -1.0;
    if (axis_linear_y_flip) vel.linear.y *= -1.0;
    if (axis_linear_z_flip) vel.linear.z *= -1.0;
    if (axis_angular_x_flip) pose.linear.x *= -1.0;
    if (axis_angular_y_flip) pose.linear.y *= -1.0;
    if (axis_angular_z_flip) vel.angular.z *= -1.0;
  }
  else if(joy->buttons[Right_button]==1)			//if right button pressed 
  {
    ROS_INFO("right button");
    
    vel.linear.x = joy->axes[axis_linear_x];
    vel.linear.y = joy->axes[axis_linear_y];
    vel.linear.z = joy->axes[axis_linear_z];
    pose.angular.x = joy->axes[axis_angular_x];
    pose.angular.y = joy->axes[axis_angular_y];
    vel.angular.z = joy->axes[axis_angular_z];
    
    if (axis_linear_x_flip) vel.linear.x *= -1.0;
    if (axis_linear_y_flip) vel.linear.y *= -1.0;
    if (axis_linear_z_flip) vel.linear.z *= -1.0;
    if (axis_angular_x_flip) pose.angular.x *= -1.0;
    if (axis_angular_y_flip) pose.angular.y *= -1.0;
    if (axis_angular_z_flip) vel.angular.z *= -1.0;
  }    
  else						//Proceed with normal control
  {
    vel.linear.x = joy->axes[axis_linear_x];
    vel.linear.y = joy->axes[axis_linear_y];
    vel.linear.z = joy->axes[axis_linear_z];
    vel.angular.x = joy->axes[axis_angular_x];
    vel.angular.y = joy->axes[axis_angular_y];
    vel.angular.z = joy->axes[axis_angular_z];

    if (axis_linear_x_flip) vel.linear.x *= -1.0;
    if (axis_linear_y_flip) vel.linear.y *= -1.0;
    if (axis_linear_z_flip) vel.linear.z *= -1.0;
    if (axis_angular_x_flip) vel.angular.x *= -1.0;
    if (axis_angular_y_flip) vel.angular.y *= -1.0;
    if (axis_angular_z_flip) vel.angular.z *= -1.0;
  } 
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hexapod_remote");

  ros::NodeHandle n;

  // get and set peramiter defaults if perameters arn't on the param server  
  n.param("hexapod_remote/axis_linear_x", axis_linear_x, 0);
  n.param("hexapod_remote/axis_linear_y", axis_linear_y, 1);
  n.param("hexapod_remote/axis_linear_z", axis_linear_z, 2);
  n.param("hexapod_remote/axis_angular_x", axis_angular_x, 3);
  n.param("hexapod_remote/axis_angular_y", axis_angular_y, 4);
  n.param("hexapod_remote/axis_angular_z", axis_angular_z, 5);

  n.param("hexapod_remote/A_button", A_button, 0);
  n.param("hexapod_remote/B_button", B_button, 1);
  n.param("hexapod_remote/X_button", X_button, 2);
  n.param("hexapod_remote/Y_button", Y_button, 3);
  n.param("hexapod_remote/Left_button", Left_button, 4);
  n.param("hexapod_remote/Right_button", Right_button, 5);
  n.param("hexapod_remote/Back_button", Back_button, 6);
  n.param("hexapod_remote/Start_button", Start_button, 7);
  n.param("hexapod_remote/Logitech_button", Logitech_button, 8);
  n.param("hexapod_remote/Left_joy_button", Left_joy_button, 9);
  n.param("hexapod_remote/Right_joy_button", Right_joy_button, 10);

  n.param("hexapod_remote/axis_linear_x_flip", axis_linear_x_flip, true);
  n.param("hexapod_remote/axis_linear_y_flip", axis_linear_y_flip, false);
  n.param("hexapod_remote/axis_linear_z_flip", axis_linear_z_flip, false);
  n.param("hexapod_remote/axis_angular_x_flip", axis_angular_x_flip, true); 
  n.param("hexapod_remote/axis_angular_y_flip", axis_angular_y_flip, false);
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
  //status publisheri
  ros::Publisher start_state_pub = n.advertise<std_msgs::Bool>("start_state", 1);

  //setup default variable values
  start_state.data = false;

  while(ros::ok())
  {  //do maths
  
    //publish stuff
    velocity_pub.publish(vel);
    pose_pub.publish(pose);
    start_state_pub.publish(start_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
