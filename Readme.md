Hexapod Remote Node

This node takes /joy commands and converts them into unit vectors for desired velocity, relative pose and commands for the robot. This noide should be launched with the startup of the robot as it potentialy will triger launches and mapping/locolisation.

Future work: finish the node!

Topics: 

	/desired_velocity_unit_vector
	/desired_pose_unit_vector
	/joystic_commands

Mapping:

	/joy: (Mode: X)
		axis
			1	-Left Joy Stick (left/right, Unit vector:1.0,-1.0) 
			2	-Left Joy Stick	(Up/Down, Unit vector:1.0,-1.0)
			3	-Left Triger (Vector starts at 0.0 apon first pull goes to -1.0 depressed and 1.0 released)
			4	-Right Joy Stick (left/right, Unit vector:1.0,-1.0)
			5	-Left Joy Stick	(Up/Down, Unit vector:1.0,-1.0)
			6	-Right Triger (Vector starts at 0.0 apon first pull goes to -1.0 depressed and 1.0 released)
			7	-Direction pad (left/right,Unit singular 1.0,-1.0)
			8	-Direction pad (Up/Down,Unit singular 1.0,-1.0)
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


Below is a list of expexted opperation while using the joystick. this is subject to change through out development.

Buttons:

	Start		- Start/stop walker toggle
	Back		-
	Logitech	-
	A(Green)	- Gait select
	B(Red)		- 
	X(Blue) 	- Option 1 Toggle
	Y(yellow)	- Option 2 Toggle

No buttons depressed:

	Left joystick 
		up/down		- Foward/Backward
		Left/Right	- crab Left/Right 	

	Right joystick
		up/down		- N/A
		Left/Right	- Rotate Left/Right

While LB:

	Left joystick 
		up/down		- Foward/Backward
		Left/Right	- crab Left/Right 	

	Right joystick
		up/down		- Height Up/down
		Left/Right	- Rotate Left/Right

While RB:

	Left joystick 
		up/down		- Pose Foward/Backward
		Left/Right	- Pose crab Left/Right 	

	Right joystick
		up/down		- Pose Height up
		Left/Right	- Pose Yaw Left/Right

While LB & RB:
		
	Left joystick 
		up/down		- Pose Pitch Foward/Back
		Left/Right	- Pose Roll Left/Right 	

	Right joystick
		up/down		- Pose Height up/down
		Left/Right	- Pose Yaw Left/Right

