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
    * Leg Manipulation Mode: Whilst in leg manipulation, inverts z-axis tip velocity input for primary selected leg.
* Right Bumper:
    * Leg Selection Mode: Cycles through possible leg selections for secondary leg manipulation.
    * Leg Manipulation Mode: Whilst in leg manipulation, inverts z-axis tip velocity input for secondary selected leg.

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

* Left Trigger: Controls z-axis velocity input for primary leg manipulation (inverted by Left Bumper).
* Right Trigger: Controls z-axis velocity input for secondary leg manipulation (inverted by Right Bumper).

### Joysticks:

* Left Joystick:
    * Linear Body Velocity Input Mode: Commands desired linear body velocity.
        * Up/Down: Positive/negative velocity input in the x-axis of the robot frame. (i.e. robot forward/backward)
        * Left/Right: Positive/negative velocity input in the y-axis of the robot frame. (i.e. robot left/right)
    * Primary Leg Manipulation Mode: If the primary selected leg is toggled for leg manipulation - commands desired tip velocity with respect to the robot frame.
        * Up/Down: Positive/negative velocity input in the x-axis of the robot frame. (i.e. tip forward/backward)
        * Left/Right: Positive/negative velocity input in the y-axis of the robot frame. (i.e. tip left/right)

* Right Joystick:
    * Angular Body Velocity Input Mode: Commands desired angular body velocity.
        * Up/Down: UNASSIGNED
        * Left/Right: Positive/negative angular velocity input about the z-axis of the robot frame. (i.e. robot turn left/right)
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
### Joypad:
* /joy

### Tablet PC:
* /android/joy
* /android/sensor

### Autonomous Velocity Input:
* /syropod_auto_navigation/desired_velocity

## Outputs:

### Syropod High-Level Controller Outputs
* /syropod_remote/system_state
* /syropod_remote/robot_state
* /syropod_remote/desired_velocity
* /syropod_remote/desired_pose
* /syropod_remote/posing_mode
* /syropod_remote/pose_reset_mode
* /syropod_remote/gait_selection
* /syropod_remote/cruise_control_mode
* /syropod_remote/auto_navigation_mode
* /syropod_remote/primary_leg_selection
* /syropod_remote/primary_leg_state
* /syropod_remote/primary_tip_velocity
* /syropod_remote/secondary_leg_selection
* /syropod_remote/secondary_leg_state
* /syropod_remote/secondary_tip_velocity
* /syropod_remote/parameter_selection
* /syropod_remote/parameter_adjustment

## Changelog:

Note: Version control commenced at v0.5.0. No changes were logged before this version.

- v0.5.0
    - Renamed from hexapod_remote to syropod_remote inline with release of SHC v0.5.0.

------------------------------------------------------------------------------------------------------------------------



