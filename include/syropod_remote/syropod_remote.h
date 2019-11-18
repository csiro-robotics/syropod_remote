////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_REMOTE_H
#define SYROPOD_REMOTE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <syropod_remote/AndroidSensor.h>
#include <syropod_remote/AndroidJoy.h>

#include <syropod_highlevel_controller/standard_includes.h>
#include <syropod_highlevel_controller/parameters_and_states.h>

#define NO_ROTATION 0.0
#define ENABLE_ROTATION 1.0
#define ROTATE_CLOCKWISE -1.0
#define ROTATE_COUNTERCLOCKWISE 1.0

#define DEAD_ZONE 0.05

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Designation for buttons in the joypad.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum JoypadButtonIndex
{
  A_BUTTON,
  B_BUTTON,
  X_BUTTON,
  Y_BUTTON,
  LEFT_BUMPER,
  RIGHT_BUMPER,
  BACK,
  START,
  LOGITECH,
  LEFT_JOYSTICK,
  RIGHT_JOYSTICK,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Designation for axes in the joypad.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum JoypadAxisIndex
{
  PRIMARY_X,
  PRIMARY_Y,
  PRIMARY_Z,
  SECONDARY_X,
  SECONDARY_Y,
  SECONDARY_Z,
  DPAD_LEFT_RIGHT,
  DPAD_UP_DOWN,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Designation for interface types.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum InterfaceType
{
  JOYPAD,
  TABLET_JOY,
  TABLET_SENSOR,
  KEYBOARD,
  UNASSIGNED = -1,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Designation for tip velocity input modes.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum TipVelocityInputMode
{
  XY_MODE,
  ZY_MODE,
  TIP_VELOCITY_INPUT_MODE_COUNT,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This structure contains data associated with configurable parameters of the remote.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct SyropodRemoteParameters 
{
  Parameter<int> publish_rate;
  Parameter<bool> invert_compass;
  Parameter<bool> invert_imu;
  Parameter<int> imu_sensitivity;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles generation and publishing of control messages for the syropod highlevel controller from the 
/// joypad inputs.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Remote
{
public:
  /// Contructor for remote object.
  Remote(void);
  
  /// Accessor for system state.
  /// @return Current state of the system
  inline SystemState getSystemState(void) { return system_state_; };

  /// Modifier for leg count.
  /// @param[in] count The value to be set as the leg count
  inline void setLegCount(int count) { leg_count_ = count; };

  /// Reset konami code to zero.
  inline void resetKonamiCode(void) { konami_code_ = 0; };
  
  /// Update system state to the press of Logitech button.
  void updateSystemState(void);
  
  /// Check Konami Code (Up, Up, Down, Down, Left, Right, Left, Right, B, A).
  void checkKonamiCode(void);
  
  /// Increment robot state to the press of Start button and decrement robot state to the press of back button.
  void updateRobotState(void);
  
  /// Cycle gaits to the press of A button.
  void updateGaitSelection(void);

  /// Cycle cruise control mode to the press of X button.
  void updateCruiseControlMode(void);
  
  /// Cycle auto navigation mode to the press of Y button.
  void updatePlannerMode(void);

  /// Cycle posing mode to the press of B button.
  void updatePosingMode(void);
  
  /// If no leg is currently selected, set pose reset mode depending on current posing mode instead to the press of R3.
  void updatePoseResetMode(void);

  /// Cycle parameter selection on left/right dpad press and increment/decrement selected parameter on up/down dpad.
  void updateParameterAdjustment(void);
  
  /// Cycle primary leg selection to the press of Left Bumper (L1) button.
  void updatePrimaryLegSelection(void);

  /// Cycle secondary leg selection to the press of Right Bumper (R1) button.
  void updateSecondaryLegSelection(void);

  /// Cycle primary leg state of the selected leg to the press of Left Joystick (L3) Button.
  void updatePrimaryLegState(void);

  /// Cycle secondary leg state of the selected leg to the press of Right Joystick (R3) Button.
  void updateSecondaryLegState(void);

  /// Update desired velocity according to the input from the user interface.
  void updateDesiredVelocity(void);

  /// Update desired pose according to the input from the user interface.
  void updateDesiredPose(void);

  /// Update tip velocity modes according to the input from the user interface.
  void updateTipVelocityModes(void);

  /// Update primary tip velocity according to the input from the user interface.
  void updatePrimaryTipVelocity(void);

  /// Update secondary tip velocity according to the input from the user interface.
  void updateSecondaryTipVelocity(void);
  
  /// Apply dead zone to joystick input axis
  /// @param[in] axis Axis to apply deadbanding to.
  void applyDeadZone(geometry_msgs::Point* axis);

  /// Apply dead zone to joystick input axes.
  /// @param[in] joy Joy message with axes to apply deadbanding to.
  void applyDeadZone(sensor_msgs::Joy* joy);
  
  /// Reset messages to zero.
  void resetMessages(void);

  /// Publish messages to the relevant topics.
  void publishMessages(void);
  
  /// Callback handling joypad input.
  /// @param[in] joy Message input from the joystick on the topic "/joy"
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /// Callback handling keyboard input.
  /// @param[in] key Message input from the keyboard on the topic "/key"
  void keyCallback(const sensor_msgs::Joy::ConstPtr& key);

  /// Callback handling android virtual joypad control of syropod.
  /// @param[in] control Touchpad joystick input data structure
  void androidJoyCallback(const syropod_remote::AndroidJoy::ConstPtr& control);

  /// Callback handling android imu sensor control of syropod.
  /// @param[in] control Touchpad accelerometer input data structure
  void androidSensorCallback(const syropod_remote::AndroidSensor::ConstPtr& control);
  
  /// Body velocity data from external source.
  /// @param[in] twist Input body velocity message data
  void externalBodyVelocityCallback(const geometry_msgs::Twist &twist);
  
  /// Pose velocity data from external source.
  /// @param[in] twist Input pose velocity message data
  void externalPoseVelocityCallback(const geometry_msgs::Twist &twist);

private:
  SyropodRemoteParameters params_; ///< Data structure containing configurable parameter values for the remote
  
  sensor_msgs::Joy joypad_control_;                       ///< Joypad input data structure
  syropod_remote::AndroidJoy android_joy_control_;        ///< Touchpad joystick input data structure
  syropod_remote::AndroidSensor android_sensor_control_;  ///< Touchpad accelerometer input data structure
  
  ros::Subscriber android_sensor_sub_;             ///< Subscriber for topic "/android/sensor"
  ros::Subscriber android_joy_sub_;                ///< Subscriber for topic "/android/joy"
  ros::Subscriber joypad_sub_;                     ///< Subscriber for topic "/joy"
  ros::Subscriber keyboard_sub_;                   ///< Subscriber for topic "/key"
  
  ros::Subscriber external_body_velocity_sub_;     ///< Subscriber for topic "/syropod_remote/external_body_velocity"
  ros::Subscriber external_pose_velocity_sub_;     ///< Subscriber for topic "/syropod_remote/external_pose_velocity"
  
  ros::Publisher desired_velocity_pub_;            ///< Publisher for topic "/syropod_remote/desired_velocity"
  ros::Publisher desired_pose_pub_;                ///< Publisher for topic "/syropod_remote/desired_pose"
  ros::Publisher primary_tip_velocity_pub_;        ///< Publisher for topic "/syropod_remote/primary_tip_velocity"
  ros::Publisher secondary_tip_velocity_pub_;      ///< Publisher for topic "/syropod_remote/secondary_tip_velocity"
  
  ros::Publisher system_state_pub_;                ///< Publisher for topic "/syropod_remote/system_state"
	ros::Publisher robot_state_pub_;                 ///< Publisher for topic "/syropod_remote/robot_state"
  ros::Publisher gait_selection_pub_;              ///< Publisher for topic "/syropod_remote/gait_selection"
  ros::Publisher posing_mode_pub_;                 ///< Publisher for topic "/syropod_remote/posing_mode"
  ros::Publisher cruise_control_pub_;              ///< Publisher for topic "/syropod_remote/cruise_control_mode"
  ros::Publisher auto_navigation_pub_;             ///< Publisher for topic "/syropod_remote/auto_navigation_mode"
  ros::Publisher planner_mode_pub_;                ///< Publisher for topic "/syropod_remote/planner_mode"
  ros::Publisher primary_leg_selection_pub_;       ///< Publisher for topic "/syropod_remote/primary_leg_selection"
  ros::Publisher secondary_leg_selection_pub_;     ///< Publisher for topic "/syropod_remote/secondary_leg_selection"
  ros::Publisher primary_leg_state_pub_;           ///< Publisher for topic "/syropod_remote/primary_leg_state"
  ros::Publisher secondary_leg_state_pub_;         ///< Publisher for topic "/syropod_remote/secondary_leg_state"
  ros::Publisher parameter_selection_pub_;         ///< Publisher for topic "/syropod_remote/parameter_selection"
  ros::Publisher parameter_adjustment_pub_;        ///< Publisher for topic "/syropod_remote/parameter_adjustment"
  ros::Publisher pose_reset_pub_;                  ///< Publisher for topic "/syropod_remote/pose_reset_mode" 
  
  SystemState system_state_ = SUSPENDED;                             ///< Current state of the system
  RobotState robot_state_ = PACKED;                                  ///< Current state of the robot
  GaitDesignation gait_selection_ = GAIT_UNDESIGNATED;               ///< Current gait selection for the walk cycle
  CruiseControlMode cruise_control_mode_ = CRUISE_CONTROL_OFF;       ///< Current cruise control mode
  PlannerMode planner_mode_ = PLANNER_MODE_OFF;                      ///< Current planner mode
  PosingMode posing_mode_ = NO_POSING;                               ///< Current posing mode for manual posing
  PoseResetMode pose_reset_mode_ = NO_RESET;                         ///< Current pose reset mode
  LegDesignation primary_leg_selection_ = LEG_UNDESIGNATED;          ///< Current primary leg selection
  LegDesignation secondary_leg_selection_ = LEG_UNDESIGNATED;        ///< Current secondary leg selection
  LegState primary_leg_state_ = WALKING;                             ///< Current primary leg state
  LegState secondary_leg_state_ = WALKING;                           ///< Current secondary leg state
  ParameterSelection parameter_selection_ = NO_PARAMETER_SELECTION;  ///< Currently selected adjustable parameter
  TipVelocityInputMode primary_tip_velocity_input_mode_ = XY_MODE;   ///< Current primary tip velocity input mode
  TipVelocityInputMode secondary_tip_velocity_input_mode_ = XY_MODE; ///< Current secondary tip velocity input mode
  
  geometry_msgs::Twist desired_velocity_msg_;        ///< Message published on "/syropod_remote/desired_velocity"
  geometry_msgs::Twist desired_pose_msg_;            ///< Message published on "/syropod_remote/desired_pose"
  geometry_msgs::Twist external_body_velocity_msg_;  ///< Message published on "/syropod_remote/external_body_velocity"
  geometry_msgs::Twist external_pose_velocity_msg_;  ///< Message published on "/syropod_remote/external_pose_velocity"
  
  geometry_msgs::Point primary_tip_velocity_msg_;    ///< Message published on "/syropod_remote/primary_tip_velocity"
  geometry_msgs::Point secondary_tip_velocity_msg_;  ///< Message published on "/syropod_remote/secondary_tip_velocity"
  std_msgs::Int8 system_state_msg_;                  ///< Message published on "/syropod_remote/system_state"
  std_msgs::Int8 robot_state_msg_;                   ///< Message published on "/syropod_remote/robot_state"
  std_msgs::Int8 gait_selection_msg_;                ///< Message published on "/syropod_remote/gait_selection"
  std_msgs::Int8 cruise_control_mode_msg_;           ///< Message published on "/syropod_remote/cruise_control_mode"
  std_msgs::Int8 planner_mode_msg_;                  ///< Message published on "/syropod_remote/planner_mode"
  std_msgs::Int8 auto_navigation_mode_msg_;          ///< Message published on "/syropod_remote/auto_navigation_mode"
  std_msgs::Int8 posing_mode_msg_;                   ///< Message published on "/syropod_remote/posing_mode"
  std_msgs::Int8 pose_reset_mode_msg_;               ///< Message published on "/syropod_remote/pose_Reset_mode"
  std_msgs::Int8 primary_leg_selection_msg_;         ///< Message published on "/syropod_remote/primary_leg_selection"
  std_msgs::Int8 secondary_leg_selection_msg_;       ///< Message published on "/syropod_remote/secondary_leg_selection"
  std_msgs::Int8 primary_leg_state_msg_;             ///< Message published on "/syropod_remote/primary_leg_state"
  std_msgs::Int8 secondary_leg_state_msg_;           ///< Message published on "/syropod_remote/secondary_leg_state"
  std_msgs::Int8 parameter_selection_msg_;           ///< Message published on "/syropod_remote/parameter_Selection"
  std_msgs::Int8 parameter_adjustment_msg_;          ///< Message published on "/syropod_remote/parameter_adjustment"

  // Debounce booleans for buttons
  bool debounce_logitech_ = true;          ///< Debounce boolean for logitech button
  bool debounce_start_ = true;             ///< Debounce boolean for start button
  bool debounce_back_ = true;              ///< Debounce boolean for back button
  bool debounce_a_ = true;                 ///< Debounce boolean for A button
  bool debounce_b_ = true;                 ///< Debounce boolean for B button
  bool debounce_x_ = true;                 ///< Debounce boolean for X button
  bool debounce_y_ = true;                 ///< Debounce boolean for Y button
  bool debounce_dpad_ = true;              ///< Debounce boolean for dpad button
  bool debounce_left_bumper_ = true;       ///< Debounce boolean for left bumper button
  bool debounce_right_bumper_ = true;      ///< Debounce boolean for right bumper button
  bool debounce_left_joystick_ = true;     ///< Debounce boolean for left joystick button
  bool debounce_right_joystick_ = true;    ///< Debounce boolean for right joystick button

  int leg_count_;                          ///< Number of legs in the robot

  int konami_code_ = 0;                    ///< Current konami code
  
  std::string current_priority_interface_;      ///< Current priority interface for controlling the robot
  std::string default_priority_interface_;      ///< Default priority interface for controlling the robot
  bool priority_interface_overridden_ = false;         ///< Flag denoting priority interface has been overridden
  InterfaceType current_interface_type_ = UNASSIGNED;  ///< Current interface type for controlling the robot
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // SYROPOD_REMOTE_H

