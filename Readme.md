Hexapod Remote Node

This node takes /Android/joy or Android/Sensor commands and converts them into unit vectors for desired velocity, relative pose and commands for the robot. This node should be launched with the startup of the robot as it potentially will trigger launches and mapping/locolisation.

Future work: finish the node!

Topics: 

	/desired_velocity
	/desired_pose

Dependancies
============

Internal:Tablet_control https://ahn005@bitbucket.csiro.au/scm/hex/tablet_control.git
