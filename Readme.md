#Hexapod Remote Node

This node takes /android/joy or android/sensor commands and converts them into unit vectors for desired velocity, relative pose and commands for the robot. This node should be launched with the startup of the robot as it potentially will trigger launches and mapping/locolisation.

Future work: finish the node!

##Please read readme of Tablet_control to know how to control hexapod with android device

##Topics: 

###subcribing

	/android/joy
	/android/sensor

###publishing

	/desired_velocity
	/desired_pose


##Dependancies

Internal: Tablet_control https://ahn005@bitbucket.csiro.au/scm/hex/tablet_control.git


