#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Joy.h"
#include "syropod_remote/androidSensor.h"
#include "syropod_remote/androidJoy.h"

// define constant which is related to rotation. These are used only in sensor-type UI.
#define NOT_ROTATE 0.0
#define ENABLE_ROTATION 1.0
#define ROTATE_CLOCKWISE -1.0
#define ROTATE_COUNTERCLOCKWISE 1.0

#define NUM_SYSTEM_STATES 2
#define NUM_ROBOT_STATES 3
#define NUM_GAIT_SELECTIONS 4
#define NUM_POSING_MODES 4
#define NUM_CRUISE_CONTROL_MODES 2
#define NUM_AUTO_NAVIGATION_MODES 2
#define NUM_PARAMETER_SELECTIONS 9
#define NUM_LEG_STATES 2


enum SystemState
{
	SUSPENDED,
	OPERATIONAL,
};

enum RobotState 
{
  PACKED,
  READY,
  RUNNING,
	UNKNOWN = -1,
  OFF = -2,
};

enum GaitDesignation
{
  WAVE_GAIT,
  AMBLE_GAIT,
  RIPPLE_GAIT,
  TRIPOD_GAIT,
  GAIT_UNDESIGNATED = -1,
};

enum PosingMode
{
  NO_POSING,
  X_Y_POSING,
  PITCH_ROLL_POSING,
  Z_YAW_POSING,
};

enum CruiseControlMode
{
  CRUISE_CONTROL_OFF,
  CRUISE_CONTROL_ON,  
};

enum AutoNavigationMode
{
  AUTO_NAVIGATION_OFF,
  AUTO_NAVIGATION_ON,  
};

enum PoseResetMode
{
  NO_RESET,
  Z_YAW_RESET,
  X_Y_RESET,
  PITCH_ROLL_RESET,
  ALL_RESET,
};

enum ParameterSelection
{
  NO_PARAMETER_SELECTION,
  STEP_FREQUENCY,
  STEP_CLEARANCE,
  BODY_CLEARANCE,
  LEG_SPAN_SCALE,
  VIRTUAL_MASS,
  VIRTUAL_STIFFNESS,
  VIRTUAL_DAMPING,
  FORCE_GAIN,
};

enum LegSelection
{
  LEG_0,
  LEG_1,
  LEG_2,
  LEG_3,
  LEG_4,
  LEG_5,
  LEG_6,
  LEG_7,
  LEG_UNDESIGNATED = -1,
};

enum LegState
{
  WALKING,
  MANUAL,
};

//Modes/status'
SystemState systemState = SUSPENDED;
RobotState robotState = PACKED;
GaitDesignation gaitSelection = GAIT_UNDESIGNATED;
PosingMode posingMode = NO_POSING;
CruiseControlMode cruiseControlMode = CRUISE_CONTROL_OFF;
AutoNavigationMode autoNavigationMode = AUTO_NAVIGATION_OFF;
ParameterSelection parameterSelection = NO_PARAMETER_SELECTION;
LegSelection primaryLegSelection = LEG_UNDESIGNATED;
LegSelection secondaryLegSelection = LEG_UNDESIGNATED;
LegState primaryLegState = WALKING;
LegState secondaryLegState = WALKING;
PoseResetMode poseResetMode = NO_RESET;

std::string syropod_type;
int num_legs = 6; //default 6

bool manualPrimaryZInvert = false;
bool manualSecondaryZInvert = false;

//Messages
geometry_msgs::Twist bodyVelocityMsg;
geometry_msgs::Twist poseMsg; 
geometry_msgs::Point primaryTipVelocityMsg;
geometry_msgs::Point secondaryTipVelocityMsg;

std_msgs::Int8 systemStateMsg;
std_msgs::Int8 robotStateMsg;
std_msgs::Int8 gaitSelectionMsg;
std_msgs::Int8 posingModeMsg;
std_msgs::Int8 cruiseControlModeMsg;
std_msgs::Int8 autoNavigationModeMsg;
std_msgs::Int8 primaryLegSelectionMsg;
std_msgs::Int8 secondaryLegSelectionMsg;
std_msgs::Int8 primaryLegStateMsg;
std_msgs::Int8 secondaryLegStateMsg;
std_msgs::Int8 poseResetModeMsg;
std_msgs::Int8 parameterSelectionMsg;
std_msgs::Int8 parameterAdjustmentMsg;

//Message indices for input axes (joystick left/right)
int primaryInputAxisX = 0;
int primaryInputAxisY = 0;
int primaryInputAxisZ = 0;
int secondaryInputAxisX = 0;
int secondaryInputAxisY = 0;
int secondaryInputAxisZ = 0;

//Message indices for dpad axes
int dPadUpDown = 0;
int dPadLeftRight = 0;

//Message indices for face Buttons
int buttonA = 0;
int buttonB = 0;
int buttonX;
int buttonY;

//Message indices for bumper buttons (L1/R1)
int bumperLeft; 
int bumperRight;

//Message indices for joystick buttons (L3/R3)
int joyButtonLeft;
int joyButtonRight;

//Message indices for Start/Back buttons
int backButton;
int startButton;
int logitechButton;

//Inversion booleans for control axes
bool invertPrimaryAxisX;
bool invertPrimaryAxisY;
bool invertPrimaryAxisZ;
bool invertSecondaryAxisX;
bool invertSecondaryAxisY;
bool invertSecondaryAxisZ;

//Debounce booleans for buttons
bool debounceLogitech = true;
bool debounceStart = true;
bool debounceBack = true;
bool debounceA = true;
bool debounceB = true;
bool debounceX = true;
bool debounceY = true;
bool debounceDPad = true;
bool debounceBumperRight = true;
bool debounceBumperLeft = true;
bool debounceJoyLeft = true;
bool debounceJoyRight = true;

//Miscelanous control variables
int publishRate;
bool invertCompass;
bool invertImu;
bool correctPrimaryTrigger = true;
bool correctSecondaryTrigger = true;
int imuSensitivity;
int parameterAdjustmentSensitivity;
int konamiCode = 0;

/***********************************************************************************************************************
  * Joy callback
***********************************************************************************************************************/
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	/***********************************************************************************************************************
    * Logitech button
  ***********************************************************************************************************************/
	if (joy->buttons[logitechButton] && debounceLogitech)
	{
		int nextSystemState = (static_cast<int>(systemState)+1)%NUM_SYSTEM_STATES;
		systemState = static_cast<SystemState>(nextSystemState);
		debounceLogitech = false;
	}
	else if (!joy->buttons[logitechButton])
	{		
		debounceLogitech = true;
	}
	
	/***********************************************************************************************************************
    * Prevent state changes if system is suspended + Konami Code (Approved by Alberto)
  ***********************************************************************************************************************/
	if (systemState == SUSPENDED)
	{
		if ((konamiCode == 0 && joy->axes[dPadUpDown] == 1) ||
			  (konamiCode == 1 && joy->axes[dPadUpDown] == 1) ||
				(konamiCode == 2 && joy->axes[dPadUpDown] == -1) ||
				(konamiCode == 3 && joy->axes[dPadUpDown] == -1) ||
				(konamiCode == 4 && joy->axes[dPadLeftRight] == 1) ||
				(konamiCode == 5 && joy->axes[dPadLeftRight] == -1) ||
				(konamiCode == 6 && joy->axes[dPadLeftRight] == 1) ||
				(konamiCode == 7 && joy->axes[dPadLeftRight] == -1) ||
				(konamiCode == 8 && joy->buttons[buttonB] == 1) ||
				(konamiCode == 9 && joy->buttons[buttonA] == 1))
		{
			konamiCode++;
		}
		else if (konamiCode == 10)
		{
			std::string command_string = "play " + ros::package::getPath(syropod_type + "_syropod") + "/.easter_egg.mp3 -q";
			system(command_string.c_str());
			konamiCode = 0;
		}
		return;
	}
	else
	{
		konamiCode = 0;
	}	

  /***********************************************************************************************************************
    * Start/Back buttons
  ***********************************************************************************************************************/
  //On Start button press, iterate system state forward
  if (joy->buttons[startButton] && debounceStart) //Start button
  {
    int nextRobotState = std::min((static_cast<int>(robotState)+1), NUM_ROBOT_STATES-1);
    robotState = static_cast<RobotState>(nextRobotState);
    debounceStart = false;
  }
  else if (!joy->buttons[startButton])
  {
    debounceStart = true;
  }
  
  //On Back button press, iterate system state backward
  if (joy->buttons[backButton] && debounceBack) //Back button
  {
    int nextRobotState = std::max((static_cast<int>(robotState)-1), 0);
    robotState = static_cast<RobotState>(nextRobotState);
    debounceBack = false;
  }	
  else if (!joy->buttons[backButton])
  {
    debounceBack = true;
  }
 
  /***********************************************************************************************************************
  * Face Buttons
  ***********************************************************************************************************************/
  //Cycle gaits on A button press
  if (joy->buttons[buttonA] && debounceA)
  {    
    int nextGaitSelection = (static_cast<int>(gaitSelection)+1)%NUM_GAIT_SELECTIONS;
    gaitSelection = static_cast<GaitDesignation>(nextGaitSelection);
    debounceA = false;
  }  
  else if (!joy->buttons[buttonA])
  {
    debounceA = true;
  }
  
  //Cycle posing mode on B button press
  if (joy->buttons[buttonB] && debounceB)
  {
    int nextPosingMode = (static_cast<int>(posingMode)+1)%NUM_POSING_MODES;		
    posingMode = static_cast<PosingMode>(nextPosingMode);
		if (secondaryLegState != MANUAL)
		{
			secondaryLegSelection = LEG_UNDESIGNATED;
		}
    debounceB = false;
  }
  else if (!joy->buttons[buttonB])
  {
    debounceB = true;
  }
  
  //Cycle cruise control mode on X button press
  if (joy->buttons[buttonX] && debounceX)
  {
    int nextCruiseControlMode = (static_cast<int>(cruiseControlMode)+1)%NUM_CRUISE_CONTROL_MODES;
    cruiseControlMode = static_cast<CruiseControlMode>(nextCruiseControlMode);
    debounceX = false;
  }
  else if (!joy->buttons[buttonX])
  {
    debounceX = true;
  }
  
  //Cycle auto navigation mode on Y button press
  if (joy->buttons[buttonY] && debounceY)
  {    
    int nextAutoNavigationMode = (static_cast<int>(autoNavigationMode)+1)%NUM_AUTO_NAVIGATION_MODES;
    autoNavigationMode = static_cast<AutoNavigationMode>(nextAutoNavigationMode);
    debounceY = false;
    bodyVelocityMsg.linear.x = 0.0;
    bodyVelocityMsg.linear.y = 0.0;
    bodyVelocityMsg.angular.z = 0.0;
  }  
  else if (!joy->buttons[buttonY])
  {
    debounceY = true;
  }

  /***********************************************************************************************************************
  * D-Pad Buttons
  ***********************************************************************************************************************/
  parameterAdjustmentMsg.data = joy->axes[dPadUpDown]; //Message with 1.0 or -1.0 to increment/decrement parameter
  
  //Cycle parameter selction on left/right dpad press
  if (joy->axes[dPadLeftRight] && debounceDPad)
  {
    int nextParameterSelection = (static_cast<int>(parameterSelection)-int(joy->axes[dPadLeftRight]))%NUM_PARAMETER_SELECTIONS;
    if (nextParameterSelection < 0)
    {
      nextParameterSelection += NUM_PARAMETER_SELECTIONS;
    }
    parameterSelection = static_cast<ParameterSelection>(nextParameterSelection);
    debounceDPad = false;
  }
  else if (!joy->buttons[dPadLeftRight])
  {
    debounceDPad = true;
  }

  /***********************************************************************************************************************
   * Bumper (L1/R1) Buttons
  ***********************************************************************************************************************/
  //Cycle primary leg selection on R1 button press (skip slection if already allocated to secondary)
  if (primaryLegState == WALKING)
  {
    if (joy->buttons[bumperLeft] && debounceBumperLeft)
    {
      int nextPrimaryLegSelection = (static_cast<int>(primaryLegSelection)+1)%(num_legs+1);
      if (nextPrimaryLegSelection == static_cast<int>(secondaryLegSelection) && secondaryLegState == MANUAL)
      {
	nextPrimaryLegSelection = (static_cast<int>(secondaryLegSelection)+1)%(num_legs+1);
      }
      
      if (nextPrimaryLegSelection < num_legs)
      {
	primaryLegSelection = static_cast<LegSelection>(nextPrimaryLegSelection);
      }
      else
      {
	primaryLegSelection = LEG_UNDESIGNATED;
      }
      
      debounceBumperLeft = false;
    }
    else if (!joy->buttons[bumperLeft])
    {
      debounceBumperLeft = true;
    }
  }
  //If selected leg is in manual state, on R1 held, invert z direction
  else
  {
    manualPrimaryZInvert = static_cast<bool>(joy->buttons[bumperLeft]);
  }
  
  //Cycle secondary leg selection on L1 button press (skip slection if already allocated to primary)
  if (secondaryLegState == WALKING)
  {     
    if (joy->buttons[bumperRight] && debounceBumperRight)
    {
      int nextSecondaryLegSelection = (static_cast<int>(secondaryLegSelection)+1)%(num_legs+1);
      if (nextSecondaryLegSelection == static_cast<int>(primaryLegSelection) && primaryLegState == MANUAL)
      {
	nextSecondaryLegSelection = (static_cast<int>(primaryLegSelection)+1)%(num_legs+1);
      }
      
      if (nextSecondaryLegSelection < num_legs)
      {
	secondaryLegSelection = static_cast<LegSelection>(nextSecondaryLegSelection);
      }
      else
      {
	secondaryLegSelection = LEG_UNDESIGNATED;
      }
      
      debounceBumperRight = false;
    }
    else if (!joy->buttons[bumperRight])
    {
      debounceBumperRight = true;
    }
  }
  //If selected leg is in manual state, on L1 held, invert z direction
  else
  {
    manualSecondaryZInvert = static_cast<bool>(joy->buttons[bumperRight]);
  }
  
  
  /***********************************************************************************************************************
  * Joystick (L3/R3) Buttons
  ***********************************************************************************************************************/
  
  //On L3 button press, cycle primary leg state of selected leg
  if (primaryLegSelection != LEG_UNDESIGNATED)
  {
    if (joy->buttons[joyButtonLeft] && debounceJoyLeft)
    {
      int nextPrimaryLegState = (static_cast<int>(primaryLegState)+1)%NUM_LEG_STATES;
      primaryLegState = static_cast<LegState>(nextPrimaryLegState);
      //If 2nd leg selection same as 1st whilst 1st is toggling state, then iterate 2nd leg selection
      if (secondaryLegSelection == primaryLegSelection) 
      {
	int nextSecondaryLegSelection = (static_cast<int>(primaryLegSelection)+1)%(num_legs);
	secondaryLegSelection = static_cast<LegSelection>(nextSecondaryLegSelection);
      }
      debounceJoyLeft = false;
    }
    else if (!joy->buttons[joyButtonRight])
    {
      debounceJoyLeft = true;
    }
  }
  else
  {
    //TBD Docking procedure
    /*
    if (joy->buttons[joyButtonLeft] && debounceJoyLeft)
    {
      int nextDockingState = (static_cast<int>(dockingState)+1)%NUM_DOCKING_STATES;
      dockingState = static_cast<DockingState>(nextDockingState);
      debounceJoyLeft = false;
    }
    else if (!joy->buttons[joyButtonLeft])
    {
      debounceJoyLeft = true;
    }
    */    
  }
  
  //On R3 button press, cycle primary leg state of selected leg
  if (secondaryLegSelection != LEG_UNDESIGNATED)
  {
    if (joy->buttons[joyButtonRight] && debounceJoyRight)
    {
      int nextSecondaryLegState = (static_cast<int>(secondaryLegState)+1)%NUM_LEG_STATES;
      secondaryLegState = static_cast<LegState>(nextSecondaryLegState);
      //If 1st leg selection same as 2nd whilst 2ndst is toggling state, then iterate 1st leg selection
      if (secondaryLegSelection == primaryLegSelection) 
      {
	int nextPrimaryLegSelection = (static_cast<int>(secondaryLegSelection)+1)%(num_legs);
	primaryLegSelection = static_cast<LegSelection>(nextPrimaryLegSelection);
      }
      debounceJoyRight = false;
    }
    else if (!joy->buttons[joyButtonRight])
    {
      debounceJoyRight = true;
    }
    posingMode = NO_POSING;
  }
  //On R3 button press, if no leg is currently selected, set pose reset mode depending on current posing mode instead
  else 
  {
    if (joy->buttons[joyButtonRight])
    {
      switch(posingMode)
      {
	case(NO_POSING):
	  poseResetMode = ALL_RESET;
	  break;
	  break;
	case(X_Y_POSING):
	  poseResetMode = X_Y_RESET;
	  break;
	case(PITCH_ROLL_POSING):
	  poseResetMode = PITCH_ROLL_RESET;
	  break;
	case(Z_YAW_POSING):
	  poseResetMode = Z_YAW_RESET;
	  break;
      }
    }
    else
    {
      poseResetMode = NO_RESET;
    }    
  }

  /***********************************************************************************************************************
  * Primary/Secondary Control Axes (Left/Right Joystick)
  ***********************************************************************************************************************/
  
  //Trigger axes from /joy are defaulted to zero until the first trigger pull,
  //This corrects this and then changes the range from 1.0->-1.0 to 0.0->1.0.
  double correctedPrimaryAxisZ;
  double correctedSecondaryAxisZ;
  if (joy->axes[primaryInputAxisZ] == 0.0 && correctPrimaryTrigger)
  {
    correctedPrimaryAxisZ = 0.0;
  }
  else
  {
    correctedPrimaryAxisZ = -(joy->axes[primaryInputAxisZ] - 1.0)/2.0;
    correctPrimaryTrigger = false;
  }
  
  if (joy->axes[secondaryInputAxisZ] == 0.0 && correctSecondaryTrigger)
  {
    correctedSecondaryAxisZ = 0.0;
  }
  else
  {
    correctedSecondaryAxisZ = -(joy->axes[secondaryInputAxisZ] - 1.0)/2.0;
    correctSecondaryTrigger = false;  
  }
  
  //Turn off all velocity and pose inputs from joystick if autoNavigation is on
  if (autoNavigationMode == AUTO_NAVIGATION_OFF) 
  {
    //Reset message values
    bodyVelocityMsg.linear.x = 0.0;
    bodyVelocityMsg.linear.y = 0.0;
    bodyVelocityMsg.linear.z = 0.0;
    bodyVelocityMsg.angular.x = 0.0;
    bodyVelocityMsg.angular.y = 0.0;
    bodyVelocityMsg.angular.z = 0.0;
    poseMsg.linear.x = 0.0;
    poseMsg.linear.y = 0.0;
    poseMsg.linear.z = 0.0;	
    poseMsg.angular.x = 0.0;
    poseMsg.angular.y = 0.0;
    poseMsg.angular.z = 0.0; 
    primaryTipVelocityMsg.x = 0.0;
    primaryTipVelocityMsg.y = 0.0;
    primaryTipVelocityMsg.z = 0.0;
    secondaryTipVelocityMsg.x = 0.0;
    secondaryTipVelocityMsg.y = 0.0;
    secondaryTipVelocityMsg.z = 0.0;
    
    //Primary Control Axis (Left joystick) --Does not change with posing mode
    if (primaryLegState == MANUAL)
    {
      primaryTipVelocityMsg.x = joy->axes[primaryInputAxisY]*(invertPrimaryAxisY ? -1.0:1.0);
      primaryTipVelocityMsg.y = joy->axes[primaryInputAxisX]*(invertPrimaryAxisX ? -1.0:1.0);
      primaryTipVelocityMsg.z = correctedPrimaryAxisZ*(invertPrimaryAxisZ!=manualPrimaryZInvert ? -1.0:1.0);
    }
    else if (primaryLegState == WALKING)
    {
      bodyVelocityMsg.linear.x = joy->axes[primaryInputAxisY]*(invertPrimaryAxisY ? -1.0:1.0);
      bodyVelocityMsg.linear.y = joy->axes[primaryInputAxisX]*(invertPrimaryAxisX ? -1.0:1.0);
    }    
    
    //Secondary Control Axis (Right joystick)  --Only changes with posing mode if in walking state
    if (secondaryLegState == MANUAL)
    {
      secondaryTipVelocityMsg.x = joy->axes[secondaryInputAxisY]*(invertSecondaryAxisY ? -1.0:1.0);
      secondaryTipVelocityMsg.y = joy->axes[secondaryInputAxisX]*(invertSecondaryAxisX ? -1.0:1.0);
      secondaryTipVelocityMsg.z = correctedSecondaryAxisZ*(invertSecondaryAxisZ!=manualSecondaryZInvert ? -1.0:1.0);
    }
    else if (secondaryLegState == WALKING)
    {
      switch(posingMode)
      {
	case(NO_POSING):
	{
	  //Secondary Control Axis (Right joystick)  
	  bodyVelocityMsg.angular.z = joy->axes[secondaryInputAxisX]*(invertSecondaryAxisX ? -1.0:1.0);
	  break;
	}
	case(X_Y_POSING):
	{
	  //Secondary Control Axis (Right joystick)
	  poseMsg.linear.x = joy->axes[secondaryInputAxisY]*(invertSecondaryAxisY ? -1.0:1.0);
	  poseMsg.linear.y = joy->axes[secondaryInputAxisX]*(invertSecondaryAxisX ? -1.0:1.0);
	  break;
	}
	case(PITCH_ROLL_POSING):
	{
	  poseMsg.angular.x = -joy->axes[secondaryInputAxisX]*(invertSecondaryAxisX ? -1.0:1.0);
	  poseMsg.angular.y = joy->axes[secondaryInputAxisY]*(invertSecondaryAxisY ? -1.0:1.0);
	  break;
	}
	case(Z_YAW_POSING):
	{
	  poseMsg.linear.z = joy->axes[secondaryInputAxisY]*(invertSecondaryAxisY ? -1.0:1.0);	
	  poseMsg.angular.z = joy->axes[secondaryInputAxisX]*(invertSecondaryAxisX ? -1.0:1.0);  
	  break;
	}
      }
    }
  }    
} 

/***********************************************************************************************************************
  * Callback for android virtual joypad control of syropod
  * 	Note: Currently ported from control scheme designed for logitech joypad. 
  * 	Needs a redesign to make full use of a tablet based virtual controller
***********************************************************************************************************************/ 
void androidJoyCallback(const syropod_remote::androidJoy::ConstPtr& control)
{
  systemState = static_cast<SystemState>(control->systemState.data);
  gaitSelection = static_cast<GaitDesignation>(control->gaitSelection.data);
  posingMode = static_cast<PosingMode>(control->posingMode.data);
  cruiseControlMode = static_cast<CruiseControlMode>(control->cruiseControlMode.data);
  autoNavigationMode = static_cast<AutoNavigationMode>(control->autoNavigationMode.data);
  primaryLegSelection = static_cast<LegSelection>(control->primaryLegSelection.data);
  secondaryLegSelection = static_cast<LegSelection>(control->secondaryLegSelection.data);
  primaryLegState = static_cast<LegState>(control->primaryLegState.data);
  secondaryLegState = static_cast<LegState>(control->secondaryLegState.data);
  parameterSelection = static_cast<ParameterSelection>(control->parameterSelection.data);
  
  parameterAdjustmentMsg.data = control->parameterAdjustment.data; //Should be 0.0, 1.0 or -1.0

  if (autoNavigationMode == AUTO_NAVIGATION_OFF) 
  {
    //Reset message values
    bodyVelocityMsg.linear.x = 0.0;
    bodyVelocityMsg.linear.y = 0.0;
    bodyVelocityMsg.linear.z = 0.0;
    bodyVelocityMsg.angular.x = 0.0;
    bodyVelocityMsg.angular.y = 0.0;
    bodyVelocityMsg.angular.z = 0.0;
    poseMsg.linear.x = 0.0;
    poseMsg.linear.y = 0.0;
    poseMsg.linear.z = 0.0;	
    poseMsg.angular.x = 0.0;
    poseMsg.angular.y = 0.0;
    poseMsg.angular.z = 0.0; 
    primaryTipVelocityMsg.x = 0.0;
    primaryTipVelocityMsg.y = 0.0;
    primaryTipVelocityMsg.z = 0.0;
    secondaryTipVelocityMsg.x = 0.0;
    secondaryTipVelocityMsg.y = 0.0;
    secondaryTipVelocityMsg.z = 0.0;
    
    //Primary Control Axis --Does not change with posing mode
    if (primaryLegState == MANUAL)
    {
      primaryTipVelocityMsg.x = control->primaryControlAxis.y*(invertPrimaryAxisY ? -1.0:1.0);
      primaryTipVelocityMsg.y = control->primaryControlAxis.x*(invertPrimaryAxisX ? -1.0:1.0);
      primaryTipVelocityMsg.z = control->primaryControlAxis.z*(invertPrimaryAxisZ!=manualPrimaryZInvert ? -1.0:1.0);
    }
    else if (primaryLegState == WALKING)
    {
      bodyVelocityMsg.linear.x = control->primaryControlAxis.y*(invertPrimaryAxisY ? -1.0:1.0);
      bodyVelocityMsg.linear.y = control->primaryControlAxis.x*(invertPrimaryAxisX ? -1.0:1.0);
    }    
    
    //Secondary Control Axis (Right joystick)  --Only changes with posing mode if in walking state
    if (secondaryLegState == MANUAL)
    {
      secondaryTipVelocityMsg.x = control->secondaryControlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);
      secondaryTipVelocityMsg.y = control->secondaryControlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);
      secondaryTipVelocityMsg.z = control->secondaryControlAxis.z*(invertSecondaryAxisZ!=manualSecondaryZInvert ? -1.0:1.0);
    }
    else if (secondaryLegState == WALKING)
    {
      switch(posingMode)
      {
	case(NO_POSING):
	{
	  //Secondary Control Axis (Right joystick)  
	  bodyVelocityMsg.angular.z = -control->secondaryControlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);
	  break;
	}
	case(X_Y_POSING):
	{
	  //Secondary Control Axis (Right joystick)
	  poseMsg.linear.x = control->secondaryControlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);
	  poseMsg.linear.y = control->secondaryControlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);
	  break;
	}
	case(PITCH_ROLL_POSING):
	{
	  poseMsg.angular.x = -control->secondaryControlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);
	  poseMsg.angular.y = control->secondaryControlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);
	  break;
	}
	case(Z_YAW_POSING):
	{
	  poseMsg.linear.z = control->secondaryControlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);	
	  poseMsg.angular.z = control->secondaryControlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);  
	  break;
	}
      }
    }
  }      
}

/***********************************************************************************************************************
  * Callback for android imu sensor control of syropod
  * 	Note: Currently ported from control scheme designed for logitech joypad. 
  * 	Needs a redesign to make full use of a tablet based virtual controller
***********************************************************************************************************************/ 
void androidSensorCallback(const syropod_remote::androidSensor::ConstPtr& control)
{
  float orientationX    = 0.0;
  float orientationY    = 0.0;
  float relativeCompass = 0.0;
  float rotate          = 0.0;

  //Logic regarding deciding syropod's start/stop 
  systemState = static_cast<SystemState>(control->systemState.data);

  //Logic regarding deciding syropod's moving(Walk Foward/Backward & Strafe Left/Right)
  orientationX = 0 + round(control->orientation.x/90*imuSensitivity)/imuSensitivity;
  orientationY = 0 + round(control->orientation.y/90*imuSensitivity)/imuSensitivity;

  //Invert according to parameters
  orientationX *= (invertPrimaryAxisX!=invertImu ? -1.0:1.0);
  orientationY *= (invertPrimaryAxisY!=invertImu ? -1.0:1.0);

  //Zero values exceeding limit
  if (std::abs(orientationY) > 1.0)
  {
    orientationY = 0;
  }

  //Logic regarding deciding syropod's rotation 
  relativeCompass = control->relativeCompass.data*(invertCompass ? -1.0:1.0);

  //Rotate as required
  if (relativeCompass > 0.3 && (std::abs(orientationX)+std::abs(orientationY)<0.3))
  {
    rotate = ROTATE_COUNTERCLOCKWISE;
  }
  else if (relativeCompass<-0.3 && (std::abs(orientationX)+std::abs(orientationY)<0.3)) 
  {
    rotate = ROTATE_CLOCKWISE;
  }
  else
  {
    rotate = NOT_ROTATE;
  }

  if (autoNavigationMode == AUTO_NAVIGATION_OFF) 
  {
    //Reset message values
    bodyVelocityMsg.linear.x = 0.0;
    bodyVelocityMsg.linear.y = 0.0;
    bodyVelocityMsg.linear.z = 0.0;
    bodyVelocityMsg.angular.x = 0.0;
    bodyVelocityMsg.angular.y = 0.0;
    bodyVelocityMsg.angular.z = 0.0;
    poseMsg.linear.x = 0.0;
    poseMsg.linear.y = 0.0;
    poseMsg.linear.z = 0.0;	
    poseMsg.angular.x = 0.0;
    poseMsg.angular.y = 0.0;
    poseMsg.angular.z = 0.0; 
    primaryTipVelocityMsg.x = 0.0;
    primaryTipVelocityMsg.y = 0.0;
    primaryTipVelocityMsg.z = 0.0;
    secondaryTipVelocityMsg.x = 0.0;
    secondaryTipVelocityMsg.y = 0.0;
    secondaryTipVelocityMsg.z = 0.0;
    
    //Primary Control Axis --Does not change with posing mode
    if (primaryLegState == WALKING)
    {
      bodyVelocityMsg.linear.x = orientationY*(invertPrimaryAxisY ? -1.0:1.0);
      bodyVelocityMsg.linear.y = orientationX*(invertPrimaryAxisX ? -1.0:1.0);
    }    
    
    //Secondary Control Axis (Right joystick)
    if (secondaryLegState == WALKING)
    {
      switch(posingMode)
      {
	case(NO_POSING):
	{
	  //Secondary Control Axis (Right joystick)  
	  bodyVelocityMsg.angular.z = -rotate*(invertSecondaryAxisX ? -1.0:1.0);
	  break;
	}
	case(X_Y_POSING):
	{
	  //Secondary Control Axis (Right joystick)
	  poseMsg.linear.x = control->controlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);
	  poseMsg.linear.y = control->controlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);
	  break;
	}
	case(PITCH_ROLL_POSING):
	{
	  poseMsg.angular.x = -control->controlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);
	  poseMsg.angular.y = control->controlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);
	  break;
	}
	case(Z_YAW_POSING):
	{
	  poseMsg.linear.z = control->controlAxis.y*(invertSecondaryAxisY ? -1.0:1.0);	
	  poseMsg.angular.z = control->controlAxis.x*(invertSecondaryAxisX ? -1.0:1.0);  
	  break;
	}
      }
    }
  }      
  return;
}

/***********************************************************************************************************************
  * Body velocity data from auto navigation node
***********************************************************************************************************************/ 
void autoNavigationCallback(const geometry_msgs::Twist &twist)
{
  if (autoNavigationMode == AUTO_NAVIGATION_ON)
  {
    //Coordination frame remapping between autoNav and SHC
    bodyVelocityMsg.linear.x = twist.linear.x;
    bodyVelocityMsg.linear.y = twist.linear.y;
    bodyVelocityMsg.angular.z = twist.angular.z;
  }
}

/***********************************************************************************************************************
  * Main
***********************************************************************************************************************/ 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "syropod_remote");

  ros::NodeHandle n;
  
  parameterAdjustmentMsg.data = 0.0;

  //Get (or set defaults for) parameters defining joy array indices for buttons/axes
  n.param("syropod_remote/primary_input_axis_x", primaryInputAxisX, 0);
  n.param("syropod_remote/primary_input_axis_y", primaryInputAxisY, 1);
  n.param("syropod_remote/primary_input_axis_z", primaryInputAxisZ, 2);
  n.param("syropod_remote/secondary_input_axis_x", secondaryInputAxisX, 3);
  n.param("syropod_remote/secondary_input_axis_y", secondaryInputAxisY, 4);
  n.param("syropod_remote/secondary_input_axis_z", secondaryInputAxisZ, 5);
  
  n.param("syropod_remote/dpad_left_right", dPadLeftRight, 6);
  n.param("syropod_remote/dpad_up_down", dPadUpDown, 7);     

  n.param("syropod_remote/A_button", buttonA, 0);
  n.param("syropod_remote/B_button", buttonB, 1);
  n.param("syropod_remote/X_button", buttonX, 2);
  n.param("syropod_remote/Y_button", buttonY, 3);
  
  n.param("syropod_remote/Left_button", bumperLeft, 4);
  n.param("syropod_remote/Right_button", bumperRight, 5);
  
  n.param("syropod_remote/Back_button", backButton, 6);
  n.param("syropod_remote/Start_button", startButton, 7);
  n.param("syropod_remote/Logitech_button", logitechButton, 8);
  
  n.param("syropod_remote/Left_joy_button", joyButtonLeft, 9);
  n.param("syropod_remote/Right_joy_button", joyButtonRight, 10);    

  //Get (or set defaults for) parameters defining axes inversion
  n.param("syropod_remote/primary_input_axis_x_flip", invertPrimaryAxisX, false);
  n.param("syropod_remote/primary_input_axis_y_flip", invertPrimaryAxisY, false);
  n.param("syropod_remote/primary_input_axis_z_flip", invertPrimaryAxisZ, false);
  n.param("syropod_remote/secondary_input_axis_x_flip", invertSecondaryAxisX, false); 
  n.param("syropod_remote/secondary_input_axis_y_flip", invertSecondaryAxisY, false);
  n.param("syropod_remote/secondary_input_axis_z_flip", invertSecondaryAxisZ, false);
  
  //Get (or set defaults for) parameters for other operating variables
  n.param("syropod_remote/sensitivity", imuSensitivity, 10);
  n.param("syropod_remote/pub_rate",publishRate, 50);
  n.param("syropod_remote/compass_flip",invertCompass, true);
  n.param("syropod_remote/imu_flip", invertImu, true);
  n.param("syropod_remote/param_adjust_sensitivity", parameterAdjustmentSensitivity, 10);
	
	std::vector<std::string> leg_id_array;
	if (!n.getParam("/syropod/parameters/leg_id", leg_id_array))
	{
		ROS_ERROR("Error reading parameter/s leg_id from rosparam. Check config file is loaded and type is correct\n");
	}
	else 
	{
		num_legs = leg_id_array.size();
	}	
	
	if (!n.getParam("/syropod/parameters/syropod_type", syropod_type))
	{
		ROS_ERROR("Error reading parameter/s syropod_type from rosparam. Check config file is loaded and type is correct\n");
		syropod_type = "Unknown";
	}

  //Setup publish loop_rate 
  ros::Rate loopRate(publishRate);

  //Subscribe to control topic/s
  ros::Subscriber androidSensorSub = n.subscribe("android/sensor", 1, androidSensorCallback);
  ros::Subscriber androidJoySub = n.subscribe("android/joy", 1, androidJoyCallback);
  ros::Subscriber joypadSub = n.subscribe("joy", 1, joyCallback);
  ros::Subscriber autoNavigationSub = n.subscribe("syropod_auto_navigation/desired_velocity", 1, autoNavigationCallback);
  
  //Setup publishers 
  ros::Publisher bodyVelocityPublisher = n.advertise<geometry_msgs::Twist>("syropod_remote/desired_velocity",1);
  ros::Publisher posePublisher = n.advertise<geometry_msgs::Twist>("syropod_remote/desired_pose",1);
  ros::Publisher primaryTipVelocityPublisher = n.advertise<geometry_msgs::Point>("syropod_remote/primary_tip_velocity",1);
  ros::Publisher secondaryTipVelocityPublisher = n.advertise<geometry_msgs::Point>("syropod_remote/secondary_tip_velocity",1);
  
  //Status publishers
  ros::Publisher systemStatePublisher = n.advertise<std_msgs::Int8>("syropod_remote/system_state", 1);
	ros::Publisher robotStatePublisher = n.advertise<std_msgs::Int8>("syropod_remote/robot_state", 1);
  ros::Publisher gaitSelectionPublisher = n.advertise<std_msgs::Int8>("syropod_remote/gait_selection", 1);
  ros::Publisher posingModePublisher = n.advertise<std_msgs::Int8>("syropod_remote/posing_mode", 1);
  ros::Publisher cruiseControlPublisher = n.advertise<std_msgs::Int8>("syropod_remote/cruise_control_mode", 1);
  ros::Publisher autoNavigationPublisher = n.advertise<std_msgs::Int8>("syropod_remote/auto_navigation_mode",1);  
  ros::Publisher primaryLegSelectionPublisher = n.advertise<std_msgs::Int8>("syropod_remote/primary_leg_selection", 1);
  ros::Publisher secondaryLegSelectionPublisher = n.advertise<std_msgs::Int8>("syropod_remote/secondary_leg_selection", 1);
  ros::Publisher primaryLegStatePublisher = n.advertise<std_msgs::Int8>("syropod_remote/primary_leg_state", 1);
  ros::Publisher secondaryLegStatePublisher = n.advertise<std_msgs::Int8>("syropod_remote/secondary_leg_state", 1);  
  ros::Publisher parameterSelectionPublisher = n.advertise<std_msgs::Int8>("syropod_remote/parameter_selection", 1);
  ros::Publisher parameterAdjustmentPublisher = n.advertise<std_msgs::Int8>("syropod_remote/parameter_adjustment", 1);  
  ros::Publisher poseResetPublisher = n.advertise<std_msgs::Int8>("syropod_remote/pose_reset_mode", 1);
	
	//Init message values
	bodyVelocityMsg.linear.x = 0.0;
	bodyVelocityMsg.linear.y = 0.0;
	bodyVelocityMsg.linear.z = 0.0;
	bodyVelocityMsg.angular.x = 0.0;
	bodyVelocityMsg.angular.y = 0.0;
	bodyVelocityMsg.angular.z = 0.0;
	poseMsg.linear.x = 0.0;
	poseMsg.linear.y = 0.0;
	poseMsg.linear.z = 0.0;	
	poseMsg.angular.x = 0.0;
	poseMsg.angular.y = 0.0;
	poseMsg.angular.z = 0.0; 
	primaryTipVelocityMsg.x = 0.0;
	primaryTipVelocityMsg.y = 0.0;
	primaryTipVelocityMsg.z = 0.0;
	secondaryTipVelocityMsg.x = 0.0;
	secondaryTipVelocityMsg.y = 0.0;
	secondaryTipVelocityMsg.z = 0.0;

  while(ros::ok())
  {      
    //Assign message values
    systemStateMsg.data = static_cast<int>(systemState);
		robotStateMsg.data = static_cast<int>(robotState);
    gaitSelectionMsg.data = static_cast<int>(gaitSelection);
    posingModeMsg.data = static_cast<int>(posingMode);
    cruiseControlModeMsg.data = static_cast<int>(cruiseControlMode);
    autoNavigationModeMsg.data = static_cast<int>(autoNavigationMode);    
    primaryLegSelectionMsg.data = static_cast<int>(primaryLegSelection);
    secondaryLegSelectionMsg.data = static_cast<int>(secondaryLegSelection);
    primaryLegStateMsg.data = static_cast<int>(primaryLegState);
    secondaryLegStateMsg.data = static_cast<int>(secondaryLegState);    
    parameterSelectionMsg.data = static_cast<int>(parameterSelection);    
    poseResetModeMsg.data = static_cast<int>(poseResetMode);
    
    //Publish messages   
    systemStatePublisher.publish(systemStateMsg);
		robotStatePublisher.publish(robotStateMsg);
    bodyVelocityPublisher.publish(bodyVelocityMsg);	
    posePublisher.publish(poseMsg);
    primaryTipVelocityPublisher.publish(primaryTipVelocityMsg);
    secondaryTipVelocityPublisher.publish(secondaryTipVelocityMsg);    
      
    gaitSelectionPublisher.publish(gaitSelectionMsg);
    posingModePublisher.publish(posingModeMsg);
    cruiseControlPublisher.publish(cruiseControlModeMsg);
    autoNavigationPublisher.publish(autoNavigationModeMsg);    
    primaryLegSelectionPublisher.publish(primaryLegSelectionMsg);
    secondaryLegSelectionPublisher.publish(secondaryLegSelectionMsg);
    primaryLegStatePublisher.publish(primaryLegStateMsg);
    secondaryLegStatePublisher.publish(secondaryLegStateMsg);    
    parameterSelectionPublisher.publish(parameterSelectionMsg);
    parameterAdjustmentPublisher.publish(parameterAdjustmentMsg);    
    poseResetPublisher.publish(poseResetModeMsg);
	    
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}
