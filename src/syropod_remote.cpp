////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "syropod_remote/syropod_remote.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Remote::Remote(void)
{
  ros::NodeHandle n;
  
  // Subscribe to control topic/s
  android_sensor_sub_ = n.subscribe("android/sensor", 1, &Remote::androidSensorCallback, this);
  android_joy_sub_ = n.subscribe("android/joy", 1, &Remote::androidJoyCallback, this);
  joypad_sub_ = n.subscribe("joy", 1, &Remote::joyCallback, this);
  keyboard_sub_ = n.subscribe("key", 1, &Remote::keyCallback, this);
  
  // External body and pose velocity topics
  external_body_velocity_sub_ = n.subscribe("syropod_remote/external_body_velocity", 1,
                                            &Remote::externalBodyVelocityCallback, this);
  external_pose_velocity_sub_ = n.subscribe("syropod_remote/external_pose_velocity", 1,
                                            &Remote::externalPoseVelocityCallback, this);

  // Setup publishers 
  desired_velocity_pub_ = n.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity",1);
  desired_pose_pub_ = n.advertise<geometry_msgs::Twist>("syropod_remote/desired_pose",1);
  primary_tip_velocity_pub_ = n.advertise<geometry_msgs::Point>("syropod_remote/primary_tip_velocity",1);
  secondary_tip_velocity_pub_ = n.advertise<geometry_msgs::Point>("syropod_remote/secondary_tip_velocity",1);
  
  // Status publishers
  system_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/system_state", 1);
	robot_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/robot_state", 1);
  gait_selection_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
  posing_mode_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/posing_mode", 1);
  cruise_control_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/cruise_control_mode", 1);
  auto_navigation_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/auto_navigation_mode",1);
  planner_mode_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/planner_mode", 1);
  primary_leg_selection_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/primary_leg_selection", 1);
  secondary_leg_selection_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/secondary_leg_selection", 1);
  primary_leg_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/primary_leg_state", 1);
  secondary_leg_state_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/secondary_leg_state", 1);
  parameter_selection_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/parameter_selection", 1);
  parameter_adjustment_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/parameter_adjustment", 1);
  pose_reset_pub_ = n.advertise<std_msgs::Int8>("syropod_remote/pose_reset_mode", 1);
  
  // Get (or set defaults for) parameters for other operating variables
  params_.imu_sensitivity.init("imu_sensitivity", "syropod_remote/");
  params_.publish_rate.init("publish_rate", "syropod_remote/");
  params_.invert_compass.init("invert_compass", "syropod_remote/");
  params_.invert_imu.init("invert_imu", "syropod_remote/");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        int next_system_state = (static_cast<int>(system_state_)+1) % SYSTEM_STATE_COUNT;
        system_state_ = static_cast<SystemState>(next_system_state);
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
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        Parameter<std::string> syropod_type;
        syropod_type.init("syropod_type", "/syropod/parameters/");
        std::string syropod_package_name = syropod_type.data + "_syropod";
        std::string command_string = "play " + ros::package::getPath(syropod_package_name) + "/.easter_egg.mp3 -q";
        system(command_string.c_str());
        konami_code_ = 0;
      }
      break;
    }
    case (TABLET_JOY):
      // TODO
      break;
    case (TABLET_SENSOR):
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateRobotState(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // On Start button press, iterate system state forward
      bool start_pressed = joypad_control_.buttons[START];
      if (start_pressed && debounce_start_) // Start button
      {
        int next_robot_state = std::min((static_cast<int>(robot_state_) + 1), ROBOT_STATE_COUNT - 1);
        robot_state_ = static_cast<RobotState>(next_robot_state);
        debounce_start_ = false;
      }
      else if (!start_pressed)
      {
        debounce_start_ = true;
      }
      
      // On Back button press, iterate system state backward
      bool back_pressed = joypad_control_.buttons[BACK];
      if (back_pressed && debounce_back_) // Back button
      {
        int next_robot_state = std::max((static_cast<int>(robot_state_)-1), 0);
        robot_state_ = static_cast<RobotState>(next_robot_state);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateGaitSelection(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Cycle gaits on A button press
      bool a_pressed = joypad_control_.buttons[A_BUTTON];
      if (a_pressed && debounce_a_)
      {    
        int next_gait_selection = (static_cast<int>(gait_selection_) + 1) % GAIT_DESIGNATION_COUNT;
        gait_selection_ = static_cast<GaitDesignation>(next_gait_selection);
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
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateCruiseControlMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Cycle cruise control mode on X button press
      bool x_pressed = joypad_control_.buttons[JoypadButtonIndex::X_BUTTON];
      if (x_pressed && debounce_x_)
      {
        int next_cruise_control_mode = (static_cast<int>(cruise_control_mode_) + 1) % CRUISE_CONTROL_MODE_COUNT;
        cruise_control_mode_ = static_cast<CruiseControlMode>(next_cruise_control_mode);
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
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updatePlannerMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Cycle auto navigation mode on Y button press
      bool y_pressed = joypad_control_.buttons[JoypadButtonIndex::Y_BUTTON];
      if (y_pressed && debounce_y_)
      {    
        int next_planner_mode = (static_cast<int>(planner_mode_)+1) % PLANNER_MODE_COUNT;
        planner_mode_ = static_cast<PlannerMode>(next_planner_mode);
        debounce_y_ = false;
      }  
      else if (!y_pressed)
      {
        debounce_y_ = true;
      }
      break;
    }
    case (TABLET_JOY):
      // TODO // auto_navigation_mode_ = static_cast<PlannerMode>(android_joy_control_.planner_mode.data);
      break;
    case (TABLET_SENSOR):
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updatePosingMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Cycle posing mode on B button press
      bool b_pressed = joypad_control_.buttons[B_BUTTON];
      if (b_pressed && debounce_b_)
      {
        int next_posing_mode = (static_cast<int>(posing_mode_)+1) % POSING_MODE_COUNT;
        posing_mode_ = static_cast<PosingMode>(next_posing_mode);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updatePoseResetMode(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // On R3 button press, if no leg is currently selected, set pose reset mode depending on current posing mode instead
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
              pose_reset_mode_ = X_AND_Y_RESET;
              break;
            case(PITCH_ROLL_POSING):
              pose_reset_mode_ = PITCH_AND_ROLL_RESET;
              break;
            case(Z_YAW_POSING):
              pose_reset_mode_ = Z_AND_YAW_RESET;
              break;
            default:
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateParameterAdjustment(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Message with 1.0 or -1.0 to increment/decrement parameter
      parameter_adjustment_msg_.data = joypad_control_.axes[JoypadAxisIndex::DPAD_UP_DOWN]; 
      
      // Cycle parameter selction on left/right dpad press
      int dpad_left_right = joypad_control_.axes[JoypadAxisIndex::DPAD_LEFT_RIGHT];
      if (dpad_left_right && debounce_dpad_)
      {
        int next_parameter_selection = (static_cast<int>(parameter_selection_) - dpad_left_right) % PARAMETER_SELECTION_COUNT;
        if (next_parameter_selection < 0)
        {
          next_parameter_selection += PARAMETER_SELECTION_COUNT;;
        }
        parameter_selection_ = static_cast<ParameterSelection>(next_parameter_selection);
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
      parameter_adjustment_msg_.data = android_joy_control_.parameter_adjustment.data; // Should be 0.0, 1.0 or -1.0
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updatePrimaryLegSelection(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Cycle primary leg selection on L1 button press (skip slection if already allocated to secondary)
      bool left_bumper_pressed = joypad_control_.buttons[JoypadButtonIndex::LEFT_BUMPER];
      if (primary_leg_state_ == WALKING)
      {
        if (left_bumper_pressed && debounce_left_bumper_)
        {
          int next_primary_leg_selection = (static_cast<int>(primary_leg_selection_)+1)%(leg_count_ +1);
          if (next_primary_leg_selection == static_cast<int>(secondary_leg_selection_) && secondary_leg_state_ == MANUAL)
          {
            next_primary_leg_selection = (static_cast<int>(secondary_leg_selection_)+1)%(leg_count_ +1);
          }
          if (next_primary_leg_selection < leg_count_)
          {
            primary_leg_selection_ = static_cast<LegDesignation>(next_primary_leg_selection);
          }
          else
          {
            primary_leg_selection_ = LEG_UNDESIGNATED;
          }
          debounce_left_bumper_ = false;
        }
        else if (!left_bumper_pressed)
        {
          debounce_left_bumper_ = true;
        }
      }
      break;
    }
    case (TABLET_JOY):
      primary_leg_selection_ = static_cast<LegDesignation>(android_joy_control_.primary_leg_selection.data);
      break;
    case (TABLET_SENSOR):
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateSecondaryLegSelection(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // Cycle secondary leg selection on R1 button press (skip slection if already allocated to primary)
      bool right_bumper_pressed = joypad_control_.buttons[JoypadButtonIndex::RIGHT_BUMPER];
      if (secondary_leg_state_ == WALKING)
      {
        if (right_bumper_pressed && debounce_right_bumper_)
        {
          int next_secondary_leg_selection = (static_cast<int>(secondary_leg_selection_)+1)%(leg_count_ +1);
          if (next_secondary_leg_selection == static_cast<int>(primary_leg_selection_) && primary_leg_state_ == MANUAL)
          {
            next_secondary_leg_selection = (static_cast<int>(primary_leg_selection_)+1)%(leg_count_ +1);
          }
          
          if (next_secondary_leg_selection < leg_count_)
          {
            secondary_leg_selection_ = static_cast<LegDesignation>(next_secondary_leg_selection);
          }
          else
          {
            secondary_leg_selection_ = LEG_UNDESIGNATED;
          }
          
          debounce_right_bumper_ = false;
        }
        else if (!right_bumper_pressed)
        {
          debounce_right_bumper_ = true;
        }
      }
      break;
    }
    case (TABLET_JOY):
      secondary_leg_selection_ = static_cast<LegDesignation>(android_joy_control_.secondary_leg_selection.data);
      break;
    case (TABLET_SENSOR):
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updatePrimaryLegState(void)
{
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // On L3 button press, cycle primary leg state of selected leg
      bool left_joystick_pressed = joypad_control_.buttons[LEFT_JOYSTICK];
      if (primary_leg_selection_ != LEG_UNDESIGNATED)
      {
        if (left_joystick_pressed && debounce_left_joystick_)
        {
          int next_primary_leg_state = (static_cast<int>(primary_leg_state_) + 1) % LEG_STATE_COUNT;
          primary_leg_state_ = static_cast<LegState>(next_primary_leg_state);
          // If 2nd leg selection same as 1st whilst 1st is toggling state, then iterate 2nd leg selection
          if (secondary_leg_selection_ == primary_leg_selection_) 
          {
            int nextSecondaryLegSelection = (static_cast<int>(primary_leg_selection_)+1)%(leg_count_);
            secondary_leg_selection_ = static_cast<LegDesignation>(nextSecondaryLegSelection);
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
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateSecondaryLegState(void)
{  
  switch (current_interface_type_)
  {
    case (KEYBOARD):
    case (JOYPAD):
    {
      // On R3 button press, cycle secondary leg state of selected leg
      bool right_joystick_pressed = joypad_control_.buttons[RIGHT_JOYSTICK];
      if (secondary_leg_selection_ != LEG_UNDESIGNATED)
      {
        if (right_joystick_pressed && debounce_right_joystick_)
        {
          int next_secondary_leg_state = (static_cast<int>(secondary_leg_state_) + 1) % LEG_STATE_COUNT;
          secondary_leg_state_ = static_cast<LegState>(next_secondary_leg_state);
          // If 1st leg selection same as 2nd whilst 2ndst is toggling state, then iterate 1st leg selection
          if (secondary_leg_selection_ == primary_leg_selection_) 
          {
            int nextPrimaryLegSelection = (static_cast<int>(secondary_leg_selection_)+1)%(leg_count_);
            primary_leg_selection_ = static_cast<LegDesignation>(nextPrimaryLegSelection);
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
      // TODO
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateDesiredVelocity(void)
{
  if (cruise_control_mode_ != CRUISE_CONTROL_EXTERNAL)
  {
    // Linear Velocity
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
        case (TABLET_SENSOR): // TODO Refactor
        {
          // Logic regarding deciding syropod's moving(Walk Foward/Backward & Strafe Left/Right)
          double s = params_.imu_sensitivity.data;
          desired_velocity_msg_.linear.y = round(android_sensor_control_.orientation.x/90.0 * s)/s;
          desired_velocity_msg_.linear.x = round(android_sensor_control_.orientation.y/90.0 * s)/s;
          // Zero values exceeding limit
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

    // Angular velocity
    if (posing_mode_ == NO_POSING && secondary_leg_state_ == WALKING)
    {
      switch (current_interface_type_)
      {
        case (KEYBOARD):
        {
          if (primary_leg_state_ == WALKING)
          {
            desired_velocity_msg_.angular.z = joypad_control_.axes[PRIMARY_Z];
          }
          break;
        }
        case (JOYPAD):
        {
          // Right Joystick control
          desired_velocity_msg_.angular.z = joypad_control_.axes[SECONDARY_X];
          break;
        }
        case (TABLET_JOY):
          desired_velocity_msg_.angular.z = android_joy_control_.secondary_control_axis.x;
          break;
        case (TABLET_SENSOR): // TODO Refactor
        {
          // Logic regarding deciding syropod's moving(Walk Foward/Backward & Strafe Left/Right)
          double sensitivity = params_.imu_sensitivity.data;
          double orientation_x = 0 + round(android_sensor_control_.orientation.x/90.0 * sensitivity)/sensitivity;
          double orientation_y = 0 + round(android_sensor_control_.orientation.y/90.0 * sensitivity)/sensitivity;
          // Zero values exceeding limit
          if (abs(orientation_y) > 1.0)
          {
            orientation_y = 0;
          }

          // Logic regarding deciding syropod's rotation 
          double compass_inverter = (params_.invert_compass.data ? -1.0 : 1.0);
          double relative_compass = android_sensor_control_.relative_compass.data * compass_inverter;

          // Rotate as required
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
  else
  {
    desired_velocity_msg_ = external_body_velocity_msg_;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateDesiredPose(void)
{
  if (secondary_leg_state_ == WALKING)
  {
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
            desired_pose_msg_.angular.x = -android_joy_control_.secondary_control_axis.x;
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
      case (EXTERNAL_POSING):
      {
        desired_pose_msg_ = external_pose_velocity_msg_;
        break;
      }
      default:
        break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateTipVelocityModes(void)
{
  if (primary_leg_state_ == MANUAL && current_interface_type_ == JOYPAD)
  {
    bool left_bumper_pressed = joypad_control_.buttons[JoypadButtonIndex::LEFT_BUMPER];
    if (left_bumper_pressed && debounce_left_bumper_)
    {
      int next_primary_tip_velocity_input_mode = 
        (static_cast<int>(primary_tip_velocity_input_mode_) + 1) % TIP_VELOCITY_INPUT_MODE_COUNT;
      primary_tip_velocity_input_mode_ = static_cast<TipVelocityInputMode>(next_primary_tip_velocity_input_mode);
      debounce_left_bumper_ = false;
    }
    else if (!left_bumper_pressed)
    {
      debounce_left_bumper_ = true;
    }
  }
  
  if (secondary_leg_state_ == MANUAL && current_interface_type_ == JOYPAD)
  {
    bool right_bumper_pressed = joypad_control_.buttons[JoypadButtonIndex::RIGHT_BUMPER];
    if (right_bumper_pressed && debounce_right_bumper_)
    {
      int next_secondary_tip_velocity_input_mode = 
        (static_cast<int>(secondary_tip_velocity_input_mode_) + 1) % TIP_VELOCITY_INPUT_MODE_COUNT;
      secondary_tip_velocity_input_mode_ = static_cast<TipVelocityInputMode>(next_secondary_tip_velocity_input_mode);
      debounce_right_bumper_ = false;
    }
    else if (!right_bumper_pressed)
    {
      debounce_right_bumper_ = true;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updatePrimaryTipVelocity(void)
{
  if (primary_leg_state_ == MANUAL) 
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
        if (primary_tip_velocity_input_mode_ == XY_MODE)
        {
          primary_tip_velocity_msg_.x = joypad_control_.axes[PRIMARY_Y];
          primary_tip_velocity_msg_.y = joypad_control_.axes[PRIMARY_X];
        }
        else if (primary_tip_velocity_input_mode_ == ZY_MODE)
        {
          primary_tip_velocity_msg_.z = joypad_control_.axes[PRIMARY_Y];
          primary_tip_velocity_msg_.y = joypad_control_.axes[PRIMARY_X];
        }
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
        // TODO
        break;
      default:
        break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::updateSecondaryTipVelocity(void)
{
  if (secondary_leg_state_ == MANUAL) 
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
        if (secondary_tip_velocity_input_mode_ == XY_MODE)
        {
          secondary_tip_velocity_msg_.x = joypad_control_.axes[SECONDARY_Y];
          secondary_tip_velocity_msg_.y = joypad_control_.axes[SECONDARY_X];
        }
        else if (secondary_tip_velocity_input_mode_ == ZY_MODE)
        {
          secondary_tip_velocity_msg_.z = joypad_control_.axes[SECONDARY_Y];
          secondary_tip_velocity_msg_.y = joypad_control_.axes[SECONDARY_X];
        }
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
        // TODO
        break;
      default:
        break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::resetMessages(void)
{
  // Init message values
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::publishMessages(void)
{
  // Assign message values
  system_state_msg_.data = static_cast<int>(system_state_);
  robot_state_msg_.data = static_cast<int>(robot_state_);
  gait_selection_msg_.data = static_cast<int>(gait_selection_);
  posing_mode_msg_.data = static_cast<int>(posing_mode_);
  cruise_control_mode_msg_.data = static_cast<int>(cruise_control_mode_);
  planner_mode_msg_.data = static_cast<int>(planner_mode_);
  primary_leg_selection_msg_.data = static_cast<int>(primary_leg_selection_);
  secondary_leg_selection_msg_.data = static_cast<int>(secondary_leg_selection_);
  primary_leg_state_msg_.data = static_cast<int>(primary_leg_state_);
  secondary_leg_state_msg_.data = static_cast<int>(secondary_leg_state_);
  parameter_selection_msg_.data = static_cast<int>(parameter_selection_);
  pose_reset_mode_msg_.data = static_cast<int>(pose_reset_mode_);
  
  // Publish messages
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
  planner_mode_pub_.publish(planner_mode_msg_);
  primary_leg_selection_pub_.publish(primary_leg_selection_msg_);
  secondary_leg_selection_pub_.publish(secondary_leg_selection_msg_);
  primary_leg_state_pub_.publish(primary_leg_state_msg_);
  secondary_leg_state_pub_.publish(secondary_leg_state_msg_);    
  parameter_selection_pub_.publish(parameter_selection_msg_);
  parameter_adjustment_pub_.publish(parameter_adjustment_msg_);    
  pose_reset_pub_.publish(pose_reset_mode_msg_);
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::applyDeadZone(sensor_msgs::Joy* joy)
{
  double primary_x = joy->axes[PRIMARY_X];
  double primary_y = joy->axes[PRIMARY_Y];
  double secondary_x = joy->axes[SECONDARY_X];
  double secondary_y = joy->axes[SECONDARY_Y];
  double primary_magnitude = sqrt((primary_x * primary_x) + (primary_y * primary_y));
  double secondary_magnitude = sqrt((secondary_x * secondary_x) + (secondary_y * secondary_y));
  double primary_x_norm = primary_x / primary_magnitude;
  double secondary_x_norm = secondary_x / secondary_magnitude;
  double primary_y_norm = primary_y / primary_magnitude;
  double secondary_y_norm = secondary_y / secondary_magnitude;
  double adjusted_primary_x = primary_x_norm * ((primary_magnitude - DEAD_ZONE) / (1 - DEAD_ZONE));
  double adjusted_primary_y = primary_y_norm * ((primary_magnitude - DEAD_ZONE) / (1 - DEAD_ZONE));
  double adjusted_secondary_x = secondary_x_norm * ((secondary_magnitude - DEAD_ZONE) / (1 - DEAD_ZONE));
  double adjusted_secondary_y = secondary_y_norm * ((secondary_magnitude - DEAD_ZONE) / (1 - DEAD_ZONE));
  bool primary_dead_zone = primary_magnitude < DEAD_ZONE;
  bool secondary_dead_zone = secondary_magnitude < DEAD_ZONE;
  joy->axes[PRIMARY_X] = primary_dead_zone ? 0.0 : adjusted_primary_x;
  joy->axes[PRIMARY_Y] = primary_dead_zone ? 0.0 : adjusted_primary_y;
  joy->axes[SECONDARY_X] = secondary_dead_zone ? 0.0 : adjusted_secondary_x;
  joy->axes[SECONDARY_Y] = secondary_dead_zone ? 0.0 : adjusted_secondary_y;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  current_priority_interface_ = "joy";
  current_interface_type_ = JOYPAD;
  priority_interface_overridden_ = true;
  sensor_msgs::Joy joy_dead_zoned = *joy;
  applyDeadZone(&joy_dead_zoned);
  joypad_control_ = joy_dead_zoned;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::keyCallback(const sensor_msgs::Joy::ConstPtr& key)
{
  current_priority_interface_ = "key";
  current_interface_type_ = KEYBOARD;
  priority_interface_overridden_ = true;
  joypad_control_ = *key; // TODO Implement keyboard_control object (if needed)
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::androidJoyCallback(const syropod_remote::AndroidJoy::ConstPtr& control)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::externalBodyVelocityCallback(const geometry_msgs::Twist &twist)
{
  if (cruise_control_mode_ != CRUISE_CONTROL_OFF)
  {
    cruise_control_mode_ = CRUISE_CONTROL_EXTERNAL;
    external_body_velocity_msg_ = twist;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Remote::externalPoseVelocityCallback(const geometry_msgs::Twist &twist)
{
  if (posing_mode_ == EXTERNAL_POSING)
  {
    external_pose_velocity_msg_ = twist;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main loop. Checks for joypad inputs, updates and publishes the messages.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "syropod_remote");

  Remote remote;
  
  Parameter<std::vector<std::string>> leg_id_array;
  leg_id_array.init("leg_id");
  remote.setLegCount(leg_id_array.data.size());

  // Setup publish loop_rate 
  Parameter<double> publish_rate;
  publish_rate.init("publish_rate", "syropod_remote/");
  ros::Rate loopRate(publish_rate.data);
  
  while(ros::ok())
  {
    remote.resetMessages();

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
      remote.updatePlannerMode();
      remote.updatePosingMode();
      remote.updatePoseResetMode();
      remote.updateParameterAdjustment();
      remote.updatePrimaryLegSelection();
      remote.updateSecondaryLegSelection();
      remote.updatePrimaryLegState();
      remote.updateSecondaryLegState();
      remote.updateDesiredVelocity();
      remote.updateDesiredPose();
      remote.updateTipVelocityModes();
      remote.updatePrimaryTipVelocity();
      remote.updateSecondaryTipVelocity();
    }

    remote.publishMessages();

    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
