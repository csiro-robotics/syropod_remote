#Hexapod Remote Node

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
	Back		- Stop walker controller and initiate shut down posing sequence
	Logitech	-
	A(Green)	- Gait select (TBD)
	B(Red)		- 
	X(Blue) 	- 
	Y(yellow)	- 

No bumper buttons depressed:

	Left joystick 
		Up/Down		- Walk Forward/Backward
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- N/A
		Left/Right	- Rotate Left/Right (Only works with linear velocity, i.e. while left joystick is giving some input)

While LB:

	Left joystick 
		Up/Down		- Walk Foward/Backward
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- Shift body Forward/Backward
		Left/Right	- Shift body Left/Right

	Right Trigger
		In		- Increase posing speed
		Out		- Decrease posing speed

	Left Trigger
		In		- Increase walking speed
		Out		- Decreased walking speed

While RB:

	Left joystick 
		Up/Down		- Walk Foward/Backward
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- Pitch Down/Up 
		Left/Right	- Roll Left/Right

	Right Trigger
		In		- Increase posing speed
		Out		- Decrease posing speed

	Left Trigger
		In		- Increase walking speed
		Out		- Decreased walking speed

While LB & RB:
		
	Left joystick 
		Up/Down		- Walk Foward/Back
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- Shift Up/Down
		Left/Right	- Yaw Left/Right

	Right Trigger
		In		- Increase posing speed
		Out		- Decrease posing speed

	Left Trigger
		In		- Increase walking speed
		Out		- Decreased walking speed

Note that any pose adjustments performed whilst holding a bumper button (LB, RB, LB+RB) will remain once the bumber button is released.
If the bumper button is then pressed again the pose will return to its default. 
i.e. Pressing RB will return the Pitch and Roll to default values but not Yaw since it is controlled via LB+RB 
