#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "hexapod_remote/androidSensor.h"
#include "hexapod_remote/androidJoy.h"

// define constant which is related to rotation. These are used only in sensor-type UI.
#define NOT_ROTATE 0.0
#define ENABLE_ROTATION 1.0
#define ROTATE_CLOCKWISE -1.0
#define ROTATE_COUNTERCLOCKWISE 1.0

//define constant which is related to pose. 
#define ROTATE_MODE 0
#define BODY_FORWARD_BACKWARD_LEFT_RIGHT_MODE 1
#define PITCH_DOWN_UP_ROLL_LEFT_RIGHT_MODE 2
#define BODY_UP_DOWN_YAW_LEFT_RIGHT_MODE 3

/**
* This node is intended to convert a joypad or android's control values into commands for hexapod robot.
* this includes velocity vector and body pose vector. along with gait type and start stop commands for mapping and localisation and system
*/

// set up variables 
geometry_msgs::Twist vel;
geometry_msgs::Twist pose; 
std_msgs::Bool start_state;
std_msgs::Int8 gait_mode;
std_msgs::Bool leg_state_toggle;
std_msgs::Int8 leg_selection;
std_msgs::Int8 param_selection;
std_msgs::Int8 param_adjust;

int axis_linear_x;
int axis_linear_y;
int axis_linear_z;
int axis_angular_x;
int axis_angular_y;
int axis_angular_z;

int dpad_up_down;
int dpad_left_right;

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

bool compass_flip;
bool imu_flip;

int pub_rate;

bool correctTriggerRight = true;
bool correctTriggerLeft = true;

int sensitivity;
int param_adjust_sensitivity;

bool debounceA = true;
bool debounceB = true;
bool debounceY = true;
bool debounceUP = true;

void androidJoyCallback(const hexapod_remote::androidJoy::ConstPtr& control)
{
    start_state.data = control->start.data;
    gait_mode.data = control->gait.data;
    param_selection.data = control->paramSelection.data;
    param_adjust.data = control->paramAdjust.data;

    //Stop processing
    if(!start_state.data){
        pose.linear.x  = 0;
        pose.linear.y  = 0;
        pose.linear.z  = 0;
        pose.angular.x = 0;
        pose.angular.y = 0;
        pose.angular.z = 0;
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        return;
    }
    vel.linear.x = control->leftJoy.x;
    vel.linear.y = control->leftJoy.y;
    vel.linear.z = control->leftJoy.z;

    /**
    * Logic regarding deciding hexapod's pose
    */
    switch(control->mode.data)
    {
        case ROTATE_MODE:
          vel.angular.x = control->rightJoy.x;
          break;
        case BODY_FORWARD_BACKWARD_LEFT_RIGHT_MODE:
          pose.linear.x  = control->rightJoy.x;
          pose.linear.y  = control->rightJoy.y;
          pose.linear.z  = 0;
          pose.angular.x = 0;
          pose.angular.y = 0;
          pose.angular.z = 0;
          break;
        case PITCH_DOWN_UP_ROLL_LEFT_RIGHT_MODE:
          pose.linear.x  = 0;
          pose.linear.y  = 0;
          pose.linear.z  = 0;
          pose.angular.x = control->rightJoy.x;
          pose.angular.y = control->rightJoy.y;
          pose.angular.z = 0;
          break;
        case BODY_UP_DOWN_YAW_LEFT_RIGHT_MODE:
          pose.linear.x  = 0;
          pose.linear.y  = 0;
          pose.linear.z  = control->rightJoy.y;
          pose.angular.x = 0;
          pose.angular.y = 0;
          pose.angular.z = -control->rightJoy.x;
          break;
    }       
}

void androidSensorCallback(const hexapod_remote::androidSensor::ConstPtr& control)
{
    float orientationX    = 0.0;
    float orientationY    = 0.0;
    float relativeCompass = 0.0;
    float rotate          = 0.0;

    /**
    * Logic regarding deciding hexapod's start/stop 
    */
    start_state.data = control->start.data;

    //Stop processing
    if(!start_state.data)
    {
        pose.linear.x  = 0;
        pose.linear.y  = 0;
        pose.linear.z  = 0;
        pose.angular.x = 0;
        pose.angular.y = 0;
        pose.angular.z = 0;
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        return;
    }
    
    /**
    * Logic regarding deciding hexapod's moving(Walk Foward/Backward & Strafe Left/Right)
    */
    orientationX = 0 + round(control->orientation.x/90*sensitivity)/sensitivity;
    orientationY = 0 + round(control->orientation.y/90*sensitivity)/sensitivity;
    
    if (axis_linear_x_flip) 
      orientationX *= -1.0;
    
    if (axis_linear_y_flip) 
      orientationY *= -1.0;
    
    if (imu_flip)
    {
      orientationX *= -1.0;
      orientationY *= -1.0;
    }
    
    // Get rid of value exceeding to limit
    if(std::abs(orientationY)>1) 
      orientationY = 0;

    /**
    * Logic regarding deciding hexapod's rotation 
    */
    relativeCompass = control->relativeCompass.data;
    if(compass_flip) 
      relativeCompass *= -1.0;

    if(relativeCompass>0.3 && (std::abs(orientationX)+std::abs(orientationY)<0.3) ) 
      rotate =ROTATE_COUNTERCLOCKWISE;
    else if(relativeCompass<-0.3 && (std::abs(orientationX)+std::abs(orientationY)<0.3)) 
      rotate =ROTATE_CLOCKWISE;
    else 
      rotate = NOT_ROTATE;

    /**
    * Logic regarding deciding hexapod's pose
    */
    switch(control->mode.data)
    {
      case BODY_FORWARD_BACKWARD_LEFT_RIGHT_MODE:
        pose.linear.x  = control->joy.x;
        pose.linear.y  = control->joy.y;
        pose.linear.z  = 0;
        pose.angular.x = 0;
        pose.angular.y = 0;
        pose.angular.z = 0;
        break;
      case PITCH_DOWN_UP_ROLL_LEFT_RIGHT_MODE:
        pose.linear.x  = 0;
        pose.linear.y  = 0;
        pose.linear.z  = 0;
        pose.angular.x = control->joy.x;
        pose.angular.y = control->joy.y;
        pose.angular.z = 0;
        break;
      case BODY_UP_DOWN_YAW_LEFT_RIGHT_MODE:
        pose.linear.x  = 0;
        pose.linear.y  = 0;
        pose.linear.z  = control->joy.y;
        pose.angular.x = 0;
        pose.angular.y = 0;
        pose.angular.z = -control->joy.x;
        break;
    }

    /**
    * Set hexapod's commands
    */
    vel.linear.z  = 1;
    vel.angular.x = rotate;
    vel.angular.z = 1;
    if(rotate){
    // to rotate, vel.linear.x or vel.linear.y should not set to 0
        vel.linear.x = ENABLE_ROTATION;
        vel.linear.y = ENABLE_ROTATION;
    }
    else
    {
        vel.linear.x = orientationX;
        vel.linear.y = orientationY;
    }
    return;
}

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
  
  //CHECK FOR B BUTTON PRESS
  if (joy->buttons[B_button] == 1)
  {
    leg_state_toggle.data = true;
  }
  else if (joy->buttons[B_button] == 0)
  {
    leg_state_toggle.data = false;
  }
  
  //CHECK FOR Y BUTTON PRESS
  if (joy->buttons[Y_button] == 1 && debounceY == true)
  {    
    leg_selection.data = (leg_selection.data+1)%6;
    debounceY = false;
  }  
  else if (joy->buttons[Y_button] == 0)
  {
    debounceY = true;
  }
  
  //CHECK FOR A BUTTON PRESS
  if (joy->buttons[A_button] == 1 && debounceA == true)
  {    
    gait_mode.data = (gait_mode.data+1)%3;
    debounceA = false;
  }  
  else if (joy->buttons[A_button] == 0)
  {
    debounceA = true;
  }
  
  //CHECK FOR D-PAD PRESS
  param_adjust.data = joy->axes[dpad_up_down];
  
  int numberOfParams = 5; //Currently only three choices TBD
  if (joy->axes[dpad_left_right] != 0)
  {
    param_selection.data = int(param_selection.data - joy->axes[dpad_left_right])%numberOfParams; 
    if (param_selection.data < 0)
    {
      param_selection.data += numberOfParams;
    }
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
  
  //Trigger axes from /joy are defaulted to zero until the first trigger pull,
  //This corrects the value initially to 1.0 as per the actual trigger position.
  if (joy->axes[axis_linear_z] == 0.0 && correctTriggerLeft)
    vel.linear.z = 1.0;
  else
    correctTriggerLeft = false;
  
  if (joy->axes[axis_angular_z] == 0.0 && correctTriggerRight)
    vel.angular.z = 1.0;
  else
    correctTriggerRight = false;  
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "hexapod_remote");

    ros::NodeHandle n;
    
    //Init values
    leg_selection.data = -1;
    gait_mode.data = -1;
    param_selection.data = 0;
    param_adjust.data = 0;

    // get and set parameter defaults if parameters arn't on the param server  
    n.param("hexapod_remote/axis_linear_x", axis_linear_x, 0);
    n.param("hexapod_remote/axis_linear_y", axis_linear_y, 1);
    n.param("hexapod_remote/axis_linear_z", axis_linear_z, 2);
    n.param("hexapod_remote/axis_angular_x", axis_angular_x, 3);
    n.param("hexapod_remote/axis_angular_y", axis_angular_y, 4);
    n.param("hexapod_remote/axis_angular_z", axis_angular_z, 5);
    
    n.param("hexapod_remote/dpad_left_right", dpad_left_right, 6);
    n.param("hexapod_remote/dpad_up_down", dpad_up_down, 7); 
    

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

    n.param("hexapod_remote/sensitivity", sensitivity, 10);
    n.param("hexapod_remote/pub_rate",pub_rate, 50);
    n.param("hexapod_remote/compass_flip",compass_flip, true);
    n.param("hexapod_remote/imu_flip", imu_flip, true);
    n.param("hexapod_remote/param_adjust_sensitivity", param_adjust_sensitivity, 10);
    

    //setup publish loop_rate 
    ros::Rate loop_rate(pub_rate); 			//parameterize  

    // subscribe to published topic
    ros::Subscriber androidSensorSub = n.subscribe("android/sensor", 1, androidSensorCallback);
    ros::Subscriber androidJoySub = n.subscribe("android/joy", 1, androidJoyCallback);
    ros::Subscriber joypadSub = n.subscribe("joy", 1, JoyCallback);
    
    //setup publishers 
    //velocity publisher
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("hexapod_remote/desired_velocity",1);
    //pose publisher
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Twist>("hexapod_remote/desired_pose",1);
    
    //status publisheri
    ros::Publisher start_state_pub = n.advertise<std_msgs::Bool>("hexapod_remote/start_state", 1);
    ros::Publisher gait_mode_pub = n.advertise<std_msgs::Int8>("hexapod_remote/gait_mode", 1);
    ros::Publisher leg_selection_pub = n.advertise<std_msgs::Int8>("hexapod_remote/leg_selection", 1);
    ros::Publisher leg_state_toggle_pub = n.advertise<std_msgs::Bool>("hexapod_remote/leg_state_toggle", 1);
    ros::Publisher param_selection_pub = n.advertise<std_msgs::Int8>("hexapod_remote/param_selection", 1);
    ros::Publisher param_adjust_pub = n.advertise<std_msgs::Int8>("hexapod_remote/param_adjust", 1);
    
    //setup default variable values
    start_state.data = false;

    while(ros::ok())
    {
        //publish stuff
        velocity_pub.publish(vel);
        pose_pub.publish(pose);
        start_state_pub.publish(start_state);
        gait_mode_pub.publish(gait_mode);
        leg_selection_pub.publish(leg_selection);
        leg_state_toggle_pub.publish(leg_state_toggle);
        param_selection_pub.publish(param_selection);
        param_adjust_pub.publish(param_adjust);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
