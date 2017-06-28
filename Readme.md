# Syropod Remote

An interface between user input via Logitech controller or Tablet PC and the Syropod High-level Controller (SHC)

Current version: v0.5.0

## Tablet Control
Please read readme of Tablet_control for Syropod control using tablet PCs.

## Joypad Control
Control scheme for Logitech F710 wireless gamepad:

### Buttons:

* Logitech: Controls SHC system state. Press to start/suspend/resume control.
* Start: Increments robot state. Press to transition forward through possible robot states. (i.e. PACKED->READY->RUNNING)
* Back: Decrements robot state. Press to transition backward through possible robot states. (i.e. RUNNING->READY->PACKED)

* A (Green): Cycles through possible gait selections defined in config/gait.yaml (defaults include Wave, Amble, Ripple and Tripod)
* B (Red): Cycles through manual body posing modes:
    * NO_POSING: Right joy stick commands NO posing input instead commands desired angular body velocity.
    * X_Y_POSING: Right joy stick commands linear translational posing in the x-axis and y-axis of the robot frame.
    * ROLL_PITCH_POSING: Right joy stick commands angular rotational posing about he x-axis (roll) and y-axis (pitch) of the robot frame.
    * Z_YAW_POSING: Right joy stick commands linear translational posing in the z-axis and angular rotational posing about the z-axis (yaw) of the robot frame.
* X (Blue): Starts/Stops cruise control mode. Cruise control sets a constant input velocity dependent on:
    * The input body velocities at activation OR
    * Parameter values defined in config/\*SYROPOD_NAME\*.yaml
* Y (Yellow): Starts/Stops auto navigation mode. Auto navigation required correct sensing capabilities and Syropod_Auto_Navigation.

* Left Bumper:
    * Leg Selection Mode: Cycles through possible leg selections for primary leg manipulation.
    * Leg Manipulation Mode: Whilst in leg manipulation, cycles between tip velocity input mode or secondary selected leg.
                             (i.e. Manipulation in X/Y or Z/Y planes)
* Right Bumper:
    * Leg Selection Mode: Cycles through possible leg selections for secondary leg manipulation.
    * Leg Manipulation Mode: Whilst in leg manipulation, cycles between tip velocity input mode or secondary selected leg.
                             (i.e. Manipulation in X/Y or Z/Y planes)

* Left Joystick Button: 
    * UNASSIGNED: If no leg is selected for primary manipulation - perform unassigned action.
    * Leg Manipulation Mode: If a leg is selected for primary manipulation - toggles leg manipulation for primary selected leg.
* Right Joystick Button: Toggles leg manipulation for secondary selected leg.
    * Pose reset mode: If no leg is selected for primary manipulation - reset all current body poses to zero (according to current POSING_MODE)
    * Leg Manipulation Mode: If a leg is selected for secondary manipulation - toggles leg manipulation for secondary selected leg.


### D-Pad:

* Left/Right: Cycles through possible adjustable parameter selections.
* Up/Down: Adjusts selected parameter by incrementing/decrementing according to adjustment step amount defined in config/\*SYROPOD_NAME\*.yaml.

### Triggers:

* Left Trigger: Commands desired angular body velocity in the positive direction (i.e. robot turn left)
* Right Trigger: Commands desired angular body velocity in the negative direction (i.e. robot turn right)

### Joysticks:

* Left Joystick:
    * Linear Body Velocity Input Mode: Commands desired linear body velocity.
        * Up/Down: Positive/negative velocity input in the x-axis of the robot frame. (i.e. robot forward/backward)
        * Left/Right: Positive/negative velocity input in the y-axis of the robot frame. (i.e. robot left/right)
    * Primary Leg Manipulation Mode: If the primary selected leg is toggled for leg manipulation - commands desired tip velocity with respect to the robot frame.
        * Up/Down: Positive/negative velocity input in the x-axis of the robot frame. (i.e. tip forward/backward)
        * Left/Right: Positive/negative velocity input in the y-axis of the robot frame. (i.e. tip left/right)

* Right Joystick:
    * No Posing Mode: Delivers no posing input.
    * X/Y Posing Mode: If the current POSING_MODE is X_Y_POSING - commands desired linear posing velocity in the x/y axes.
        * Up/Down: Positive/Negative velocity input in the x-axis of the robot frame. (i.e. pose robot body forward/backward)
        * Left/Right: Positive/Negative velocity input in the y-axis of the robot frame. (i.e. pose robot body left/right)
    * Roll/Pitch Posing Mode: If the current POSING_MODE is ROLL_PITCH_POSING - commands desired angular posing velocity about the x/y axes.
        * Up/Down: Positive/Negative velocity input about the y-axis of the robot frame. (i.e. pitch robot body forward/backward)
        * Left/Right: Negative/Positive velocity input about the x-axis of the robot frame. (i.e. roll robot body left/right)
    * Z/Yaw Posing Mode: If the current POSING_MODE is Z_YAW_POSING - commands desired linear/angular posing velocity in/about the z axis.
        * Up/Down: Positive/Negative velocity input in the z-axis of the robot frame. (i.e. pose robot body up/down)
        * Left/Right: Positive/Negative velocity input about the z-axis of the robot frame. (i.e. yaw robot body left/right)
    * Secondary Leg Manipulation Mode: If the secondary selected leg is toggled for leg manipulation - commands desired tip velocity with respect to the robot frame.
        * Up/Down: Positive/negative velocity input in the x-axis of the robot frame. (i.e. tip forward/backward)
        * Left/Right: Positive/negative velocity input in the y-axis of the robot frame. (i.e. tip left/right)

### Konami Code:

* Press in the Konami Code sequence to find out. (May not be implemented for all Syropods)

## Inputs:
* Joypad Input:
    * Description: Input message from joypad.
    * Topic: */joy*
    * Type: sensor_msgs::Joy::ConstPtr
* Android Tablet Joypad Input
    * Description: See tablet_control package.
    * Topic: */android/joy*
    * Type: syropod_remote::androidJoy::ConstPtr (custom message)
* Android Tablet Accelerometer Input
    * Description: See tablet_control package.
    * Topic: */android/sensor*
    * Type: syropod_remote::androidSensor::ConstPtr (custom message)
* Syropod Auto-Navigation Input
    * Description: The input desired body velocity from syropod_auto_navigation node. 
    * Topic: */syropod\_auto\_navigation/desired\_velocity*
    * Type: geometry_msgs::Twist

## Outputs:
* System State:
    * Description: The desired state of the entire Syropod High-level Controller system.
    * Topic: */syropod\_remote/system\_state*
    * Type: std_msgs::Int8
* Robot State:
    * Description: The desired state of the robot.
    * Topic: */syropod\_remote/robot_state*
    * Type: std_msgs::Int8
* Desired Velocity:
    * Description: The desired body velocity of the robot.
    * Topic: */syropod\_remote/desired\_velocity*
    * Type: geometry_msgs::Twist
* Desired Pose:
    * Description: The desired body pose of the robot.
    * Topic: */syropod\_remote/desired\_pose*
    * Type: geometry_msgs::Twist
* Posing Mode:
    * Description: The desired manual body posing input mode.
    * Topic: */syropod\_remote/posing\_mode*
    * Type: std_msgs::Int8
* Pose Reset Mode:
    * Description: The desired manual body pose reset mode.
    * Topic: */syropod\_remote/pose\_reset\_mode*
    * Type: std_msgs::Int8
* Gait Selection:
    * Description: The desired gait selection for the walk controller of the robot.
    * Topic: */syropod\_remote/gait\_selection*
    * Type: std_msgs::Int8
* Cruise Control Mode:
    * Description: The desired cruise control mode.
    * Topic: */syropod\_remote/cruise\_control\_mode*
    * Type: std_msgs::Int8
* Auto-Nagivation Mode:
    * Description: The desired auto-navigation mode.
    * Topic: */syropod\_remote/auto\_navigation\_mode*
    * Type: std_msgs::Int8
* Primary Leg Selection:
    * Description: The desired leg selected for primary manipulation.
    * Topic: */syropod\_remote/primary\_leg\_selection*
    * Type: std_msgs::Int8
* Primary Leg State:
    * Description: The desired state of the leg selected for primary manipulation.
    * Topic: */syropod\_remote/primary\_leg\_state*
    * Type: std_msgs::Int8
* Primary Tip Velocity:
    * Description: The desired tip velocity for the leg selected for primary manipulation.
    * Topic: */syropod\_remote/primary\_tip\_velocity*
    * Type: geometry_msgs::Point
* Secondary Leg Selection:
    * Description: The desired leg selected for secondary manipulation.
    * Topic: */syropod\_remote/secondary\_leg\_selection*
    * Type: std_msgs::Int8
* Secondary Leg State:
    * Description: The desired state of the leg selected for secondary manipulation.
    * Topic: */syropod\_remote/secondary\_leg\_state*
    * Type: std_msgs::Int8
* Secondary Tip Velocity:
    * Description: The desired tip velocity for the leg selected for secondary manipulation.
    * Topic: */syropod\_remote/secondary\_tip\_velocity*
    * Type: geometry_msgs::Point
* Parameter Selection:
    * Description: The desired parameter selection for possible adjustment.
    * Topic: */syropod\_remote/parameter\_selection*
    * Type: std_msgs::Int8
* Parameter Adjustment:
    * Description: The desired adjustment of the selected parameter (increment/decrement).
    * Topic: */syropod\_remote/parameter\_adjustment*
    * Type: std_msgs::Int8

## Changelog:

Note: Version control commenced at v0.5.0. No changes were logged before this version.

- v0.5.0
    - Renamed from hexapod_remote to syropod_remote inline with release of SHC v0.5.0.
    - Refactored into command specific functions
    - Added support for keyboard control
    - Modified control scheme for leg manipulation plane selection
    - Modified control scheme for to make joypad triggers control body angular velocity

------------------------------------------------------------------------------------------------------------------------



