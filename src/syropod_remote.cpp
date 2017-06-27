/*******************************************************************************************************************//**
 *  @file    syropod_remote.cpp
 *  @brief   Source file for Syropod Remote node.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    June 2017
 *  @version 0.5.0
 *
 *  CSIRO Autonomous Systems Laboratory
 *  Queensland Centre for Advanced Technologies
 *  PO Box 883, Kenmore, QLD 4069, Australia
 *
 *  (c) Copyright CSIRO 2017
 *
 *  All rights reserved, no part of this program may be used
 *  without explicit permission of CSIRO
 *
***********************************************************************************************************************/

#include "syropod_remote/syropod_remote.h"

Remote::Remote(ros::NodeHandle n, Parameters* params)
  : n_(n)
  , params_(params)
{
  //Subscribe to control topic/s
  android_sensor_sub_ = n_.subscribe("android/sensor", 1, &Remote::androidSensorCallback, this);
  //android_joy_sub_ = n_.subscribe("android/joy", 1, &Remote::androidJoyCallback, this);
  joypad_sub_ = n_.subscribe("joy", 1, &Remote::joyCallback, this);
  keyboard_sub_ = n_.subscribe("key", 1, &Remote::keyCallback, this);
  auto_navigation_sub_ = n_.subscribe("syropod_auto_navigation/desired_velocity", 1, 
                                      &Remote::autoNavigationCallback, this);

  //Setup publishers 
  desired_velocity_pub_ = n_.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity",1);
  desired_pose_pub_ = n_.advertise<geometry_msgs::Twist>("syropod_remote/desired_pose",1);
  primary_tip_velocity_pub_ = n_.advertise<geometry_msgs::Point>("syropod_remote/primary_tip_velocity",1);
  secondary_tip_velocity_pub_ = n_.advertise<geometry_msgs::Point>("syropod_remote/secondary_tip_velocity",1);
  
  //Status publishers
  system_state_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/system_state", 1);
	robot_state_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/robot_state", 1);
  gait_selection_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
  posing_mode_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/posing_mode", 1);
  cruise_control_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/cruise_control_mode", 1);
  auto_navigation_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/auto_navigation_mode",1);
  primary_leg_selection_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/primary_leg_selection", 1);
  secondary_leg_selection_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/secondary_leg_selection", 1);
  primary_leg_state_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/primary_leg_state", 1);
  secondary_leg_state_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/secondary_leg_state", 1);
  parameter_selection_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/parameter_selection", 1);
  parameter_adjustment_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/parameter_adjustment", 1);
  pose_reset_pub_ = n_.advertise<std_msgs::Int8>("syropod_remote/pose_reset_mode", 1);
}

/***********************************************************************************************************************
  * Logitech button
***********************************************************************************************************************/
void Remote::updateSystemState(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      bool logitech_pressed = joypad_control_.buttons[JoypadButtonIndex::LOGITECH];
      if (logitech_pressed && debounce_logitech_)
      {
        int nextSystemState = (static_cast<int>(system_state_)+1)%NUM_SYSTEM_STATES;
        system_state_ = static_cast<SystemState>(nextSystemState);
        debounce_logitech_ = false;
      }
      else if (!logitech_pressed)
      {
        debounce_logitech_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      system_state_ = static_cast<SystemState>(android_joy_control_.system_state.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Konami Code (Up, Up, Down, Down, Left, Right, Left, Right, B, A)
***********************************************************************************************************************/
void Remote::checkKonamiCode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      bool dPad_up_pressed = (joypad_control_.axes[JoypadAxisIndex::DPAD_UP_DOWN] == 1);
      bool dPad_down_pressed = (joypad_control_.axes[JoypadAxisIndex::DPAD_UP_DOWN] == -1);
      bool dPad_left_pressed = (joypad_control_.axes[JoypadAxisIndex::DPAD_LEFT_RIGHT] == 1);
      bool dPad_right_pressed = (joypad_control_.axes[JoypadAxisIndex::DPAD_LEFT_RIGHT] == -1);
      bool b_pressed = joypad_control_.buttons[JoypadButtonIndex::B_BUTTON];
      bool a_pressed = joypad_control_.buttons[JoypadButtonIndex::A_BUTTON];
      
      if ((konami_code_ == 0 && dPad_up_pressed) ||
          (konami_code_ == 1 && dPad_up_pressed) ||
          (konami_code_ == 2 && dPad_down_pressed) ||
          (konami_code_ == 3 && dPad_down_pressed) ||
          (konami_code_ == 4 && dPad_left_pressed) ||
          (konami_code_ == 5 && dPad_right_pressed) ||
          (konami_code_ == 6 && dPad_left_pressed) ||
          (konami_code_ == 7 && dPad_right_pressed) ||
          (konami_code_ == 8 && b_pressed) ||
          (konami_code_ == 9 && a_pressed))
      {
        konami_code_++;
      }
      else if (konami_code_ == 10)
      {
        Parameter<string> syropod_type;
        syropod_type.init(n_, "syropod_type", "/syropod/parameters/");
        string syropod_package_name = syropod_type.data + "_syropod";
        string command_string = "play " + ros::package::getPath(syropod_package_name) + "/.easter_egg.mp3 -q";
        system(command_string.c_str());
        konami_code_ = 0;
      }
      break;
    }
    case (TABLET_JOY):
      //TODO
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Start button
***********************************************************************************************************************/
void Remote::updateRobotState(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //On Start button press, iterate system state forward
      bool start_pressed = joypad_control_.buttons[START];
      if (start_pressed && debounce_start_) //Start button
      {
        int nextRobotState = std::min((static_cast<int>(robot_state_)+1), NUM_ROBOT_STATES-1);
        robot_state_ = static_cast<RobotState>(nextRobotState);
        debounce_start_ = false;
      }
      else if (!start_pressed)
      {
        debounce_start_ = true;
      }
      
      //On Back button press, iterate system state backward
      bool back_pressed = joypad_control_.buttons[BACK];
      if (back_pressed && debounce_back_) //Back button
      {
        int nextRobotState = std::max((static_cast<int>(robot_state_)-1), 0);
        robot_state_ = static_cast<RobotState>(nextRobotState);
        debounce_back_ = false;
      }	
      else if (!back_pressed)
      {
        debounce_back_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      robot_state_ = static_cast<RobotState>(android_joy_control_.robot_state.data);
      break;
    case (TABLET_SENSOR):
      robot_state_ = static_cast<RobotState>(android_sensor_control_.robot_state.data);
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * A Button
***********************************************************************************************************************/
void Remote::updateGaitSelection(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Cycle gaits on A button press
      bool a_pressed = joypad_control_.buttons[A_BUTTON];
      if (a_pressed && debounce_a_)
      {    
        int nextGaitSelection = (static_cast<int>(gait_selection_)+1)%NUM_GAIT_SELECTIONS;
        gait_selection_ = static_cast<GaitDesignation>(nextGaitSelection);
        debounce_a_ = false;
      }
      else if (!a_pressed)
      {
        debounce_a_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      gait_selection_ = static_cast<GaitDesignation>(android_joy_control_.gait_selection.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * X Button
***********************************************************************************************************************/
void Remote::updateCruiseControlMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Cycle cruise control mode on X button press
      bool x_pressed = joypad_control_.buttons[JoypadButtonIndex::X_BUTTON];
      if (x_pressed && debounce_x_)
      {
        int nextCruiseControlMode = (static_cast<int>(cruise_control_mode_)+1)%NUM_CRUISE_CONTROL_MODES;
        cruise_control_mode_ = static_cast<CruiseControlMode>(nextCruiseControlMode);
        debounce_x_ = false;
      }
      else if (!x_pressed)
      {
        debounce_x_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      cruise_control_mode_ = static_cast<CruiseControlMode>(android_joy_control_.cruise_control_mode.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Y Button
***********************************************************************************************************************/
void Remote::updateAutoNavigationMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Cycle auto navigation mode on Y button press
      bool y_pressed = joypad_control_.buttons[JoypadButtonIndex::Y_BUTTON];
      if (y_pressed && debounce_y_)
      {    
        int nextAutoNavigationMode = (static_cast<int>(auto_navigation_mode_)+1)%NUM_AUTO_NAVIGATION_MODES;
        auto_navigation_mode_ = static_cast<AutoNavigationMode>(nextAutoNavigationMode);
        debounce_y_ = false;
        desired_velocity_msg_.linear.x = 0.0;
        desired_velocity_msg_.linear.y = 0.0;
        desired_velocity_msg_.angular.z = 0.0;
      }  
      else if (!y_pressed)
      {
        debounce_y_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      auto_navigation_mode_ = static_cast<AutoNavigationMode>(android_joy_control_.auto_navigation_mode.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * B Button
***********************************************************************************************************************/
void Remote::updatePosingMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Cycle posing mode on B button press
      bool b_pressed = joypad_control_.buttons[B_BUTTON];
      if (b_pressed && debounce_b_)
      {
        int nextPosingMode = (static_cast<int>(posing_mode_)+1)%NUM_POSING_MODES;
        posing_mode_ = static_cast<PosingMode>(nextPosingMode);
        if (secondary_leg_state_ != MANUAL)
        {
          secondary_leg_selection_ = LEG_UNDESIGNATED;
        }
        debounce_b_ = false;
      }
      else if (!b_pressed)
      {
        debounce_b_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      posing_mode_ = static_cast<PosingMode>(android_joy_control_.posing_mode.data);
      break;
    case (TABLET_SENSOR):
      posing_mode_ = static_cast<PosingMode>(android_sensor_control_.posing_mode.data);
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Pose Reset
***********************************************************************************************************************/
void Remote::updatePoseResetMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //On R3 button press, if no leg is currently selected, set pose reset mode depending on current posing mode instead
      bool right_joystick_pressed = joypad_control_.buttons[RIGHT_JOYSTICK];
      if (secondary_leg_selection_ == LEG_UNDESIGNATED)
      {
        if (right_joystick_pressed)
        {
          switch(posing_mode_)
          {
            case(NO_POSING):
              pose_reset_mode_ = ALL_RESET;
              break;
            case(X_Y_POSING):
              pose_reset_mode_ = X_Y_RESET;
              break;
            case(PITCH_ROLL_POSING):
              pose_reset_mode_ = PITCH_ROLL_RESET;
              break;
            case(Z_YAW_POSING):
              pose_reset_mode_ = Z_YAW_RESET;
              break;
          }
        }
        else
        {
          pose_reset_mode_ = NO_RESET;
        }
      }
      break;
    }
    case (TABLET_JOY):
      pose_reset_mode_ = static_cast<PoseResetMode>(android_joy_control_.pose_reset_mode.data);
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * D-Pad Buttons
***********************************************************************************************************************/
void Remote::updateParameterAdjustment(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Message with 1.0 or -1.0 to increment/decrement parameter
      parameter_adjustment_msg_.data = joypad_control_.axes[JoypadAxisIndex::DPAD_UP_DOWN]; 
      
      //Cycle parameter selction on left/right dpad press
      int dpad_left_right = joypad_control_.axes[JoypadAxisIndex::DPAD_LEFT_RIGHT];
      if (dpad_left_right && debounce_dpad_)
      {
        int nextParameterSelection = (static_cast<int>(parameter_selection_)-dpad_left_right)%NUM_PARAMETER_SELECTIONS;
        if (nextParameterSelection < 0)
        {
          nextParameterSelection += NUM_PARAMETER_SELECTIONS;
        }
        parameter_selection_ = static_cast<ParameterSelection>(nextParameterSelection);
        debounce_dpad_ = false;
      }
      else if (!dpad_left_right)
      {
        debounce_dpad_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      parameter_selection_ = static_cast<ParameterSelection>(android_joy_control_.parameter_selection.data);
      parameter_adjustment_msg_.data = android_joy_control_.parameter_adjustment.data; //Should be 0.0, 1.0 or -1.0
      break;
    default:
      break;
  }
}



/***********************************************************************************************************************
  * Left Bumper (L1) Button
***********************************************************************************************************************/
void Remote::updatePrimaryLegSelection(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Cycle primary leg selection on R1 button press (skip slection if already allocated to secondary)
      bool left_bumped_pressed = joypad_control_.buttons[JoypadButtonIndex::LEFT_BUMPER];
      if (primary_leg_state_ == WALKING)
      {
        if (left_bumped_pressed && debounce_left_bumper_)
        {
          int nextPrimaryLegSelection = (static_cast<int>(primary_leg_selection_)+1)%(leg_count_ +1);
          if (nextPrimaryLegSelection == static_cast<int>(secondary_leg_selection_) && secondary_leg_state_ == MANUAL)
          {
            nextPrimaryLegSelection = (static_cast<int>(secondary_leg_selection_)+1)%(leg_count_ +1);
          }
          if (nextPrimaryLegSelection < leg_count_)
          {
            primary_leg_selection_ = static_cast<LegSelection>(nextPrimaryLegSelection);
          }
          else
          {
            primary_leg_selection_ = LEG_UNDESIGNATED;
          }
          debounce_left_bumper_ = false;
        }
        else if (!left_bumped_pressed)
        {
          debounce_left_bumper_ = true;
        }
      }
      break;
    }
    case (TABLET_JOY):
      primary_leg_selection_ = static_cast<LegSelection>(android_joy_control_.primary_leg_selection.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Right Bumper (R1) Button
***********************************************************************************************************************/
void Remote::updateSecondaryLegSelection(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //Cycle secondary leg selection on L1 button press (skip slection if already allocated to primary)
      bool right_bumped_pressed = joypad_control_.buttons[JoypadButtonIndex::RIGHT_BUMPER];
      if (secondary_leg_state_ == WALKING)
      {
        if (right_bumped_pressed && debounce_right_bumper_)
        {
          int nextSecondaryLegSelection = (static_cast<int>(secondary_leg_selection_)+1)%(leg_count_ +1);
          if (nextSecondaryLegSelection == static_cast<int>(primary_leg_selection_) && primary_leg_state_ == MANUAL)
          {
            nextSecondaryLegSelection = (static_cast<int>(primary_leg_selection_)+1)%(leg_count_ +1);
          }
          
          if (nextSecondaryLegSelection < leg_count_)
          {
            secondary_leg_selection_ = static_cast<LegSelection>(nextSecondaryLegSelection);
          }
          else
          {
            secondary_leg_selection_ = LEG_UNDESIGNATED;
          }
          
          debounce_right_bumper_ = false;
        }
        else if (!right_bumped_pressed)
        {
          debounce_right_bumper_ = true;
        }
      }
      break;
    }
    case (TABLET_JOY):
      secondary_leg_selection_ = static_cast<LegSelection>(android_joy_control_.secondary_leg_selection.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Left Joystick (L3) Buttons
***********************************************************************************************************************/
void Remote::updatePrimaryLegState(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //On L3 button press, cycle primary leg state of selected leg
      bool left_joystick_pressed = joypad_control_.buttons[LEFT_JOYSTICK];
      if (primary_leg_selection_ != LEG_UNDESIGNATED)
      {
        if (left_joystick_pressed && debounce_left_joystick_)
        {
          int nextPrimaryLegState = (static_cast<int>(primary_leg_state_)+1)%NUM_LEG_STATES;
          primary_leg_state_ = static_cast<LegState>(nextPrimaryLegState);
          //If 2nd leg selection same as 1st whilst 1st is toggling state, then iterate 2nd leg selection
          if (secondary_leg_selection_ == primary_leg_selection_) 
          {
            int nextSecondaryLegSelection = (static_cast<int>(primary_leg_selection_)+1)%(leg_count_);
            secondary_leg_selection_ = static_cast<LegSelection>(nextSecondaryLegSelection);
          }
          debounce_left_joystick_ = false;
        }
        else if (!left_joystick_pressed)
        {
          debounce_left_joystick_ = true;
        }
      }
      break;
    }
    case (TABLET_JOY):
      primary_leg_state_ = static_cast<LegState>(android_joy_control_.primary_leg_state.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Right Joystick (R3) Button
***********************************************************************************************************************/
void Remote::updateSecondaryLegState(void)
{  
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      //On R3 button press, cycle primary leg state of selected leg
      bool right_joystick_pressed = joypad_control_.buttons[RIGHT_JOYSTICK];
      if (secondary_leg_selection_ != LEG_UNDESIGNATED)
      {
        if (right_joystick_pressed && debounce_right_joystick_)
        {
          int nextSecondaryLegState = (static_cast<int>(secondary_leg_state_)+1)%NUM_LEG_STATES;
          secondary_leg_state_ = static_cast<LegState>(nextSecondaryLegState);
          //If 1st leg selection same as 2nd whilst 2ndst is toggling state, then iterate 1st leg selection
          if (secondary_leg_selection_ == primary_leg_selection_) 
          {
            int nextPrimaryLegSelection = (static_cast<int>(secondary_leg_selection_)+1)%(leg_count_);
            primary_leg_selection_ = static_cast<LegSelection>(nextPrimaryLegSelection);
          }
          debounce_right_joystick_ = false;
        }
        else if (!right_joystick_pressed)
        {
          debounce_right_joystick_ = true;
        }
        posing_mode_ = NO_POSING;
      }
      break;
    }
    case (TABLET_JOY):
      secondary_leg_state_ = static_cast<LegState>(android_joy_control_.secondary_leg_state.data);
      break;
    case (TABLET_SENSOR):
      //TODO
      break;
    default:
      break;
  }
}

/***********************************************************************************************************************
  * Desired Velocity
***********************************************************************************************************************/
void Remote::updateDesiredVelocity(void)
{
  if (auto_navigation_mode_ == AUTO_NAVIGATION_OFF)
  {
    //resetMessages();
    if (primary_leg_state_ == WALKING)
    {
      switch (current_interface_type_)
      {
        case (KEYBOARD):
        case (JOYPAD):
          desired_velocity_msg_.linear.x = joypad_control_.axes[PRIMARY_Y];
          desired_velocity_msg_.linear.y = joypad_control_.axes[PRIMARY_X];
          break;
        case (TABLET_JOY):
          desired_velocity_msg_.linear.x = android_joy_control_.primary_control_axis.y;
          desired_velocity_msg_.linear.y = android_joy_control_.primary_control_axis.x;
          break;
        case (TABLET_SENSOR): //TODO Refactor
        {
          //Logic regarding deciding syropod's moving(Walk Foward/Backward & Strafe Left/Right)
          double s = params_->imu_sensitivity.data;
          desired_velocity_msg_.linear.y = round(android_sensor_control_.orientation.x/90.0 * s)/s;
          desired_velocity_msg_.linear.x = round(android_sensor_control_.orientation.y/90.0 * s)/s;
          //Zero values exceeding limit
          if (abs(desired_velocity_msg_.linear.y) > 1.0)
          {
            desired_velocity_msg_.linear.y = 0;
          }
          break;
        }
        default:
          break;
      }
    }
    
    if (secondary_leg_state_ == WALKING && posing_mode_ == NO_POSING)
    {
      switch (current_interface_type_)
      {
        case (KEYBOARD):
        case (JOYPAD):
          desired_velocity_msg_.angular.z = joypad_control_.axes[SECONDARY_X];
          break;
        case (TABLET_JOY):
          desired_velocity_msg_.angular.z = -android_joy_control_.secondary_control_axis.x;
          break;
        case (TABLET_SENSOR): //TODO Refactor
        {
          //Logic regarding deciding syropod's moving(Walk Foward/Backward & Strafe Left/Right)
          double sensitivity = params_->imu_sensitivity.data;
          double orientation_x = 0 + round(android_sensor_control_.orientation.x/90.0 * sensitivity)/sensitivity;
          double orientation_y = 0 + round(android_sensor_control_.orientation.y/90.0 * sensitivity)/sensitivity;
          //Zero values exceeding limit
          if (abs(orientation_y) > 1.0)
          {
            orientation_y = 0;
          }

          //Logic regarding deciding syropod's rotation 
          double compass_inverter = (params_->invert_compass.data ? -1.0 : 1.0);
          double relative_compass = android_sensor_control_.relative_compass.data * compass_inverter;

          //Rotate as required
          double rotate;
          if (relative_compass > 0.3 && (abs(orientation_x) + abs(orientation_y) < 0.3))
          {
            rotate = ROTATE_COUNTERCLOCKWISE;
          }
          else if (relative_compass < -0.3 && (abs(orientation_x) + abs(orientation_y) < 0.3)) 
          {
            rotate = ROTATE_CLOCKWISE;
          }
          else
          {
            rotate = NO_ROTATION;
          }
          desired_velocity_msg_.angular.z = rotate;
          break;
        }
        default:
          break;
      }
    }
  }
}

/***********************************************************************************************************************
  * Desired Velocity
***********************************************************************************************************************/
void Remote::updateDesiredPose(void)
{
  if (auto_navigation_mode_ == AUTO_NAVIGATION_OFF && secondary_leg_state_ == WALKING)
  {
    //resetMessages();
    switch (posing_mode_)
    {
      case (X_Y_POSING):
      {
        switch (current_interface_type_)
        {
          case (KEYBOARD):
          case (JOYPAD):
            desired_pose_msg_.linear.x = joypad_control_.axes[SECONDARY_Y];
            desired_pose_msg_.linear.y = joypad_control_.axes[SECONDARY_X];
            break;
          case (TABLET_JOY):
            desired_pose_msg_.linear.x = android_joy_control_.secondary_control_axis.y;
            desired_pose_msg_.linear.y = android_joy_control_.secondary_control_axis.x;
            break;
          case (TABLET_SENSOR):
            desired_pose_msg_.linear.x = android_sensor_control_.control_axis.y;
            desired_pose_msg_.linear.y = android_sensor_control_.control_axis.x;
            break;
          default:
            break;
        }
        break;
      }
      case (PITCH_ROLL_POSING):
      {
        switch (current_interface_type_)
        {
          case (KEYBOARD):
          case (JOYPAD):
            desired_pose_msg_.angular.x = -joypad_control_.axes[SECONDARY_X];
            desired_pose_msg_.angular.y = joypad_control_.axes[SECONDARY_Y];
            break;
          case (TABLET_JOY):
            desired_pose_msg_.angular.x = android_joy_control_.secondary_control_axis.x;
            desired_pose_msg_.angular.y = android_joy_control_.secondary_control_axis.y;
            break;
          case (TABLET_SENSOR):
            desired_pose_msg_.angular.x = android_sensor_control_.control_axis.x;
            desired_pose_msg_.angular.y = android_sensor_control_.control_axis.y;
            break;
          default:
            break;
        }
        break;
      }
      case (Z_YAW_POSING):
      {
        switch (current_interface_type_)
        {
          case (KEYBOARD):
          case (JOYPAD):
            desired_pose_msg_.linear.z = joypad_control_.axes[SECONDARY_Y];
            desired_pose_msg_.angular.z = joypad_control_.axes[SECONDARY_X];
            break;
          case (TABLET_JOY):
            desired_pose_msg_.linear.z = android_joy_control_.secondary_control_axis.y;
            desired_pose_msg_.angular.z = android_joy_control_.secondary_control_axis.x;
            break;
          case (TABLET_SENSOR):
            desired_pose_msg_.linear.z = android_sensor_control_.control_axis.y;
            desired_pose_msg_.angular.z = android_sensor_control_.control_axis.x;
            break;
          default:
            break;
        }
        break;
      }
      default:
        break;
    }
  }
}

/***********************************************************************************************************************
  * Primary Tip Velocity
***********************************************************************************************************************/
void Remote::updatePrimaryTipVelocity(void)
{
  if (auto_navigation_mode_ == AUTO_NAVIGATION_OFF && primary_leg_state_ == MANUAL) 
  {
    //resetMessages();
    switch (current_interface_type_)
    {
      case (KEYBOARD):
      {
        primary_tip_velocity_msg_.x = joypad_control_.axes[PRIMARY_Y];
        primary_tip_velocity_msg_.y = joypad_control_.axes[PRIMARY_X];
        primary_tip_velocity_msg_.z = joypad_control_.axes[PRIMARY_Z];
        break;
      }
      case (JOYPAD):
      {
        // Correct trigger
        double corrected_primary_axis_z;
        double axis_inverter = (joypad_control_.buttons[LEFT_BUMPER] ? -1.0 : 1.0);
        if (joypad_control_.axes[PRIMARY_Z] == 0.0 && primary_z_axis_corrected_)
        {
          corrected_primary_axis_z = 0.0;
        }
        else
        {
          corrected_primary_axis_z = -(joypad_control_.axes[PRIMARY_Z] - 1.0) / 2.0;
          primary_z_axis_corrected_ = false;
        }
        primary_tip_velocity_msg_.x = joypad_control_.axes[PRIMARY_Y];
        primary_tip_velocity_msg_.y = joypad_control_.axes[PRIMARY_X];
        primary_tip_velocity_msg_.z = corrected_primary_axis_z * axis_inverter;
        break;
      }
      case (TABLET_JOY):
      {
        primary_tip_velocity_msg_.x = android_joy_control_.primary_control_axis.y;
        primary_tip_velocity_msg_.y = android_joy_control_.primary_control_axis.x;
        primary_tip_velocity_msg_.z = android_joy_control_.primary_control_axis.z;
        break;
      }
      case (TABLET_SENSOR):
        //TODO
        break;
      default:
        break;
    }
  }
}

/***********************************************************************************************************************
  * Secondary Tip Velocity
***********************************************************************************************************************/
void Remote::updateSecondaryTipVelocity(void)
{
  if (auto_navigation_mode_ == AUTO_NAVIGATION_OFF && secondary_leg_state_ == MANUAL) 
  {
    //resetMessages();
    switch (current_interface_type_)
    {
      case (KEYBOARD):
      {
        secondary_tip_velocity_msg_.x = joypad_control_.axes[SECONDARY_Y];
        secondary_tip_velocity_msg_.y = joypad_control_.axes[SECONDARY_X];
        secondary_tip_velocity_msg_.z = joypad_control_.axes[SECONDARY_Z];
        break;
      }
      case (JOYPAD):
      {
        // Correct trigger
        double corrected_secondary_axis_z;
        double axis_inverter = (joypad_control_.buttons[RIGHT_BUMPER] ? -1.0 : 1.0);
        if (joypad_control_.axes[SECONDARY_Z] == 0.0 && secondary_z_axis_corrected_)
        {
          corrected_secondary_axis_z = 0.0;
        }
        else
        {
          corrected_secondary_axis_z = -(joypad_control_.axes[SECONDARY_Z] - 1.0) / 2.0;
          secondary_z_axis_corrected_ = false;
        }
        secondary_tip_velocity_msg_.x = joypad_control_.axes[SECONDARY_Y];
        secondary_tip_velocity_msg_.y = joypad_control_.axes[SECONDARY_X];
        secondary_tip_velocity_msg_.z = corrected_secondary_axis_z * axis_inverter;
        break;
      }
      case (TABLET_JOY):
      {
        secondary_tip_velocity_msg_.x = android_joy_control_.secondary_control_axis.y;
        secondary_tip_velocity_msg_.y = android_joy_control_.secondary_control_axis.x;
        secondary_tip_velocity_msg_.z = android_joy_control_.secondary_control_axis.z;
        break;
      }
      case (TABLET_SENSOR):
        //TODO
        break;
      default:
        break;
    }
  }
}

/***********************************************************************************************************************
  * Reset Messages
***********************************************************************************************************************/
void Remote::resetMessages(void)
{
  //Init message values
	desired_velocity_msg_.linear.x = 0.0;
	desired_velocity_msg_.linear.y = 0.0;
	desired_velocity_msg_.linear.z = 0.0;
	desired_velocity_msg_.angular.x = 0.0;
	desired_velocity_msg_.angular.y = 0.0;
	desired_velocity_msg_.angular.z = 0.0;
	desired_pose_msg_.linear.x = 0.0;
	desired_pose_msg_.linear.y = 0.0;
	desired_pose_msg_.linear.z = 0.0;	
	desired_pose_msg_.angular.x = 0.0;
	desired_pose_msg_.angular.y = 0.0;
	desired_pose_msg_.angular.z = 0.0; 
	primary_tip_velocity_msg_.x = 0.0;
	primary_tip_velocity_msg_.y = 0.0;
	primary_tip_velocity_msg_.z = 0.0;
	secondary_tip_velocity_msg_.x = 0.0;
	secondary_tip_velocity_msg_.y = 0.0;
	secondary_tip_velocity_msg_.z = 0.0;
  
  parameter_adjustment_msg_.data = 0.0;
}

/***********************************************************************************************************************
  * Publish Messages
***********************************************************************************************************************/
void Remote::publishMessages(void)
{
  //Assign message values
  system_state_msg_.data = static_cast<int>(system_state_);
  robot_state_msg_.data = static_cast<int>(robot_state_);
  gait_selection_msg_.data = static_cast<int>(gait_selection_);
  posing_mode_msg_.data = static_cast<int>(posing_mode_);
  cruise_control_mode_msg_.data = static_cast<int>(cruise_control_mode_);
  auto_navigation_mode_msg_.data = static_cast<int>(auto_navigation_mode_);
  primary_leg_selection_msg_.data = static_cast<int>(primary_leg_selection_);
  secondary_leg_selection_msg_.data = static_cast<int>(secondary_leg_selection_);
  primary_leg_state_msg_.data = static_cast<int>(primary_leg_state_);
  secondary_leg_state_msg_.data = static_cast<int>(secondary_leg_state_);
  parameter_selection_msg_.data = static_cast<int>(parameter_selection_);
  pose_reset_mode_msg_.data = static_cast<int>(pose_reset_mode_);
  
  //Publish messages
  system_state_pub_.publish(system_state_msg_);
  robot_state_pub_.publish(robot_state_msg_);
  desired_velocity_pub_.publish(desired_velocity_msg_);	
  desired_pose_pub_.publish(desired_pose_msg_);
  primary_tip_velocity_pub_.publish(primary_tip_velocity_msg_);
  secondary_tip_velocity_pub_.publish(secondary_tip_velocity_msg_);    
    
  gait_selection_pub_.publish(gait_selection_msg_);
  posing_mode_pub_.publish(posing_mode_msg_);
  cruise_control_pub_.publish(cruise_control_mode_msg_);
  auto_navigation_pub_.publish(auto_navigation_mode_msg_);    
  primary_leg_selection_pub_.publish(primary_leg_selection_msg_);
  secondary_leg_selection_pub_.publish(secondary_leg_selection_msg_);
  primary_leg_state_pub_.publish(primary_leg_state_msg_);
  secondary_leg_state_pub_.publish(secondary_leg_state_msg_);    
  parameter_selection_pub_.publish(parameter_selection_msg_);
  parameter_adjustment_pub_.publish(parameter_adjustment_msg_);    
  pose_reset_pub_.publish(pose_reset_mode_msg_);
  
}

/***********************************************************************************************************************
  * Apply dead zone to joystick input axis
***********************************************************************************************************************/
void Remote::applyDeadZone(geometry_msgs::Point* axis)
{
  double axis_magnitude = sqrt((axis->x * axis->x) + (axis->y * axis->y));
  double axis_x_norm = axis->x / axis_magnitude;
  double axis_y_norm = axis->y / axis_magnitude;
  if(axis_magnitude < DEAD_ZONE)
  {
    axis->x = 0.0;
    axis->y = 0.0;
  }
  else
  {
    axis->x = axis_x_norm * ((axis_magnitude - DEAD_ZONE) / (1 - DEAD_ZONE));
    axis->y = axis_y_norm * ((axis_magnitude - DEAD_ZONE) / (1 - DEAD_ZONE));
  }
}

/***********************************************************************************************************************
  * Joy callback
***********************************************************************************************************************/
void Remote::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  current_priority_interface_ = "joy";
  current_interface_type_ = JOYPAD;
  priority_interface_overridden_ = true;
  joypad_control_ = *joy;
}

/***********************************************************************************************************************
  * Joy callback
***********************************************************************************************************************/
void Remote::keyCallback(const sensor_msgs::Joy::ConstPtr& key)
{
  current_priority_interface_ = "key";
  current_interface_type_ = KEYBOARD;
  priority_interface_overridden_ = true;
  joypad_control_ = *key; //TODO Implement keyboard_control object (if needed)
}

/***********************************************************************************************************************
  * Callback for android virtual joypad control of syropod
***********************************************************************************************************************/ 
void Remote::androidJoyCallback(syropod_remote::AndroidJoy::ConstPtr& control)
{
  // Control turning off/on overiding default interface with this interface
  if (control->override_priority_interface.data && !priority_interface_overridden_)
  {
    priority_interface_overridden_ = true;
    current_priority_interface_ = control->id_name.data;
  }
  else if (!control->override_priority_interface.data && priority_interface_overridden_)
  {
    if (control->id_name.data == current_priority_interface_)
    {
      priority_interface_overridden_ = false;
      current_priority_interface_ = default_priority_interface_;
    }
  }

  // Continue only if current interface has priority
  if (control->id_name.data == current_priority_interface_)
  {
    android_joy_control_ = *control;
    current_interface_type_ = TABLET_JOY;
    applyDeadZone(&(android_joy_control_.primary_control_axis));
    applyDeadZone(&(android_joy_control_.secondary_control_axis));
  }
}

/***********************************************************************************************************************
  * Callback for android imu sensor control of syropod
***********************************************************************************************************************/ 
void Remote::androidSensorCallback(const syropod_remote::AndroidSensor::ConstPtr& control)
{
  // Control turning off/on overiding default interface with this interface
  if (control->override_priority_interface.data && !priority_interface_overridden_)
  {
    priority_interface_overridden_ = true;
    current_priority_interface_ = control->id_name.data;
  }
  else if (!control->override_priority_interface.data && priority_interface_overridden_)
  {
    if (control->id_name.data == current_priority_interface_)
    {
      priority_interface_overridden_ = false;
      current_priority_interface_ = default_priority_interface_;
    }
  }

  // Continue only if current interface has priority
  if (control->id_name.data == current_priority_interface_)
  {
    android_sensor_control_ = *control;
    current_interface_type_ = TABLET_SENSOR;
  }
}

/***********************************************************************************************************************
  * Body velocity data from auto navigation node
***********************************************************************************************************************/ 
void Remote::autoNavigationCallback(const geometry_msgs::Twist &twist)
{
  if (auto_navigation_mode_ == AUTO_NAVIGATION_ON)
  {
    //Coordination frame remapping between autoNav and SHC
    desired_velocity_msg_.linear.x = twist.linear.x;
    desired_velocity_msg_.linear.y = twist.linear.y;
    desired_velocity_msg_.angular.z = twist.angular.z;
  }
}

/***********************************************************************************************************************
  * Main
***********************************************************************************************************************/ 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "syropod_remote");

  ros::NodeHandle n;
  
  Parameters params;
  Remote remote(n, &params);
  
  //Get (or set defaults for) parameters for other operating variables
  params.imu_sensitivity.init(n, "imu_sensitivity");
  params.publish_rate.init(n, "publish_rate");
  params.invert_compass.init(n, "invert_compass");
  params.invert_imu.init(n, "invert_imu");

  Parameter<vector<string>> leg_id_array;
  leg_id_array.init(n, "leg_id", "/syropod/parameters/");
  remote.setLegCount(leg_id_array.data.size());

  //Setup publish loop_rate 
  ros::Rate loopRate(params.publish_rate.data);
  while(ros::ok())
  {
    // Check joypad inputs
    remote.updateSystemState();
    if (remote.getSystemState() == SUSPENDED)
    {
      remote.checkKonamiCode();
    }
    else
    {
      remote.resetKonamiCode();
      remote.updateRobotState();
      remote.updateGaitSelection();
      remote.updateCruiseControlMode();
      remote.updateAutoNavigationMode();
      remote.updatePosingMode();
      remote.updatePoseResetMode();
      remote.updateParameterAdjustment();
      remote.updatePrimaryLegSelection();
      remote.updateSecondaryLegSelection();
      remote.updatePrimaryLegState();
      remote.updateSecondaryLegState();
      remote.updateDesiredVelocity();
      remote.updateDesiredPose();
      remote.updatePrimaryTipVelocity();
      remote.updateSecondaryTipVelocity();
    }
    
    remote.publishMessages();
    
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/ 
