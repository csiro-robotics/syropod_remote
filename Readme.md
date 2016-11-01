# Hexapod Remote Node

This node takes /joy, /android/joy or android/sensor commands and converts them into unit vectors for desired velocity, relative pose and commands for the robot. This node should be launched with the startup of the robot as it potentially will trigger launches and mapping/locolisation.

Topics: 

	Subcribing

	/android/joy
	/android/sensor
	/joy

	Publishing

	/desired_velocity
	/desired_pose

Tablet Control:

	Please read readme of Tablet_control to know how to control hexapod with android device

Joystick Control:

	Below is a list of expected operation while using the joystick. 
	This is subject to change throughout development.

Buttons:

	Start		- Start walk controller and initiate start up posing sequence
	Back		- Stop walker controller and initiate shut down posing sequence[^1]
	Logitech	- No function
	A(Green)	- Gait select[^2]
	B(Red)		- Leg state toggle[^3]
	X(Blue) 	- Starts/Stops walking tests[^4]
	Y(yellow)	- Switches between autonomous and manual control modes[^5]
	
	D-pad (U-D) - Parameter selection[^6]
	D-pad (L-R) - Parameter adjustment[^7]

	Left trigger  - No function
	Right trigger - No function
	
	Left joystick 
		Up/Down		- Walk Forward/Backward
		Left/Right	- Strafe Left/Right 	
		Button      - No function
	
No bumper buttons pressed:

	Right joystick
		Up/Down		- No Function
		Left/Right	- Rotate Left/Right
		Button (Hold)	- Reset all posing

While LB:

	Right joystick
		Up/Down		- Shift body Forward/Backward
		Left/Right	- Shift body Left/Right
		Button (Hold)	- Reset X/Y translation posing

While RB:

	Right joystick
		Up/Down		- Pitch Down/Up 
		Left/Right	- Roll Left/Right
		Button (Hold)	- Reset Pitch/Roll rotation posing
		

While LB & RB:

	Right joystick
		Up/Down		- Shift Up/Down
		Left/Right	- Yaw Left/Right
		Button (Hold)	- Reset Yaw rotation and Z translation posing

[^1]: Start-Up/Shut-Down sequences only operational if hexapod parameter *start_up_sequence* is set true. If false BACK button will shutdown the controller entirely.

[^2]:  Cycles through wave, amble, ripple and tripod gaits. Iterates int for gaits defined by enum in simple_hexapod_controller/standardIncludes.h

[^3]:  Toggles selected leg between WALKING state and OFF state - Currently not useable since leg selection is unoperational.

[^4]: If hexapod parameter *testing* is set true, starts/stops a constant input velocity for specified time limit. Test velocity and time limit are also defined in hexapod parameters.

[^5]:  Autonomous control relies on input on the *cmd_vel* topic. Note: Y button previously used for leg selection but has been commented out.

[^6]:  Cycles through each adjustable parameter. Iterates int for parameters defined by enum in simple_hexapod_controller/standardIncludes.h

[^7]:  Scales parameter value up or down by default 10% of original value each step. This scale is set via *param_adjust_sensitivity* parameter.



