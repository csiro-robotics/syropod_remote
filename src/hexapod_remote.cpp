#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "hexapod_remote/AndroidSensor.h"
#include "hexapod_remote/AndroidJoy.h"

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
* This node is intended to convert android's control value into comands for hexapod robot.
* this includes velocity vector and body pose vector. along with gait type and start stop commands for mapping and localisation and system
*/

// set up floatiables 
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

void AndroidJoyCallback(const hexapod_remote::AndroidJoy::ConstPtr& control){
    start_state.data = control->start.data;
    vel.linear.x = control->leftJoy.x;
    vel.linear.y = control->leftJoy.y;
    vel.linear.z = control->leftJoy.z;

    /**
    * Logic regarding deciding hexapod's pose
    */
    switch(control->mode.data){
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
        pose.angular.z = control->rightJoy.x;
        break;
    }
}

void AndroidSensorCallback(const hexapod_remote::AndroidSensor::ConstPtr& control)
{
    float orientationX    = 0.0;
    float orientationY    = 0.0;
    float relativeCompass = 0.0;
    float rotate          = 0.0;

    /**
    * Logic regarding deciding hexapod's start/stop 
    */
    start_state.data = control->start.data;

    /**
    * Logic regarding deciding hexapod's moving(Walk Foward/Backward & Strafe Left/Right)
    */
    orientationX = 0 + round(control->orientation.x/90*10)/10;
    orientationY = 0 + (-1)*round(control->orientation.y/90*10)/10;

    // Get rid of value exceeding to limit
    if(std::abs(orientationY)>1) orientationY = 0;


    /**
    * Logic regarding deciding hexapod's rotation 
    */
    relativeCompass = control->relativeCompass.data;
    if(relativeCompass>0.3 && (std::abs(orientationX)+std::abs(orientationY)<0.3) ) rotate =ROTATE_COUNTERCLOCKWISE;
    else if(relativeCompass<-0.3 && (std::abs(orientationX)+std::abs(orientationY)<0.3)) rotate =ROTATE_CLOCKWISE;
    else rotate = NOT_ROTATE;

    /**
    * Logic regarding deciding hexapod's pose
    */
    switch(control->mode.data){
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
        pose.angular.z = control->joy.x;
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
    else{
        vel.linear.x = orientationX;
        vel.linear.y = orientationY;
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

    // subscribe to published topic
    ros::Subscriber sub1 = n.subscribe("android/sensor", 1, AndroidSensorCallback);
    ros::Subscriber sub2 = n.subscribe("android/joy", 1, AndroidJoyCallback);
    //setup publishers 
    //velocity publisher
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("desired_velocity",1);
    //pose publisher
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Twist>("desired_pose",1);
    //status publisheri
    ros::Publisher start_state_pub = n.advertise<std_msgs::Bool>("start_state", 1);

    //setup default floatiable values
    start_state.data = false;

    while(ros::ok())
    {
        //publish stuff
        velocity_pub.publish(vel);
        pose_pub.publish(pose);
        start_state_pub.publish(start_state);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
