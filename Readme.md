Hexapod Remote Node

This node takes /joy commands and converts them into unit vectors for desired velocity, relative pose and commands for the robot. This node should be launched with the startup of the robot as it potentially will trigger launches and mapping/locolisation.

Future work: finish the node!

Topics: 

	/desired_velocity
	/desired_pose
	/joystick_commands

Mapping:

	/joy: (Mode: X)
		axis
			1	-Left Joy Stick (Left/Right, Unit vector: 1.0,-1.0) 
			2	-Left Joy Stick	(Up/Down, Unit vector: 1.0,-1.0)
			3	-Left Trigger (Vector starts at 0.0 upon first pull goes to -1.0 depressed and 1.0 released)
			4	-Right Joy Stick (Left/Right, Unit vector: 1.0,-1.0)
			5	-Left Joy Stick	(Up/Down, Unit vector: 1.0,-1.0)
			6	-Right Trigger (Vector starts at 0.0 upon first pull goes to -1.0 depressed and 1.0 released)
			7	-Direction pad (left/right, Unit singular: 1.0,-1.0)
			8	-Direction pad (Up/Down, Unit singular: 1.0,-1.0)
		buttons (Units 0=off 1=on)
			1	-A(green)
			2	-B(Red)
			3	-X(blue)
			4	-Y(Yellow)
			5	-LB 
			6	-RB
			7	-Back
			8	-Start
			9	-Logitech
			10	-Left Joy Button
			11	-Right Joy Button


Below is a list of expected operation while using the joystick. 
This is subject to change throughout development.

Buttons:

	Start		- Start walk controller
	Back		- Stop walker controller (TBD)
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
		Left/Right	- Rotate Left/Right

While LB:

	Left joystick 
		Up/Down		- Walk Foward/Backward
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- Shift body Forward/Backward
		Left/Right	- Shift body Left/Right

While RB:

	Left joystick 
		Up/Down		- Walk Foward/Backward
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- Pitch Down/Up 
		Left/Right	- Roll Left/Right

While LB & RB:
		
	Left joystick 
		Up/Down		- Walk Foward/Back
		Left/Right	- Strafe Left/Right 	

	Right joystick
		Up/Down		- Shift Up/Down
		Left/Right	- Yaw Left/Right

Note that any pose adjustments performed whilst holding a bumper button (LB, RB, LB+RB) will remain once the bumber button is released.
If the bumper button is then pressed again the pose will return to its default. 
i.e. Pressing RB will return the Pitch and Roll to default values but not Yaw since it is controlled via LB+RB 
