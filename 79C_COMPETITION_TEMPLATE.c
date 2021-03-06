#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in5,    fieldSwitchingPot, sensorPotentiometer)
#pragma config(Sensor, in6,    leftLineSensor, sensorLineFollower)
#pragma config(Sensor, in7,    centerLineSensor, sensorLineFollower)
#pragma config(Sensor, in8,    rightLineSensor, sensorLineFollower)
#pragma config(Sensor, dgtl4,  singleDriverJumper, sensorTouch)
#pragma config(Sensor, dgtl5,  sklAutoJumper,  sensorTouch)
#pragma config(Sensor, dgtl6,  intAutoJumper,  sensorTouch)
#pragma config(Sensor, dgtl7,  bkIsoAutoJumper, sensorTouch)
#pragma config(Sensor, dgtl8,  isoAutoJumper,  sensorTouch)
#pragma config(Sensor, dgtl10, stowAngleButton, sensorTouch)
#pragma config(Sensor, dgtl11, goalBumpButton, sensorTouch)
#pragma config(Sensor, dgtl12, autoProgButton, sensorTouch)
#pragma config(Sensor, I2C_1,  LBWEncoder,     sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  RBWEncoder,     sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  RBMEncoder,     sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           leftBackWheel, tmotorVex393HighSpeed, openLoop, reversed, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port2,           leftFrontWheel, tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port3,           leftSweeper,   tmotorVex269, openLoop)
#pragma config(Motor,  port4,           leftFrontManip, tmotorVex269, openLoop, reversed)
#pragma config(Motor,  port5,           leftBackManip, tmotorVex269, openLoop, reversed)
#pragma config(Motor,  port6,           rightBackManip, tmotorVex269, openLoop, encoder, encoderPort, I2C_3, 1000)
#pragma config(Motor,  port7,           rightFrontManip, tmotorVex269, openLoop)
#pragma config(Motor,  port8,           rightSweeper,  tmotorVex269, openLoop, reversed)
#pragma config(Motor,  port9,           rightFrontWheel, tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port10,          rightBackWheel, tmotorVex393HighSpeed, openLoop, encoder, encoderPort, I2C_2, 1000)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

#include "79C_GENERAL_COMMANDS.h"
#include "79C_SOUND_LIBRARY.h"
#include "79C_IEM_AUTO_LIBRARY.h"
#include "79C_IEM_DRIVER_LIBRARY.h"
#include "79C_LINE_FOLLOWER_LIBRARY.h"
#include "79C_AUTONOMOUS_PROGRAMS.h"
#include "79C_DRIVER_CONFIG.h"


short autoMode = 0; // Autonomous Mode

void pre_auton()
{
  bStopTasksBetweenModes = true; // Allow user-created tasks to be stopped after autonomous and driver control

  initTypedefs(); // Initialize type definitions

  if(SensorValue[fieldSwitchingPot] < 2048) // Blue zone(LEFT)
  {
      overrideDirection(); // Reverse direction
  }

  resetBothWheelEncoders(); // Resets Wheel Encoders
  resetBothManipEncoders(); // Resets Manipulator Encoders

  if(SensorValue[sklAutoJumper] == 1) // Programming Skills Autonomous
  {
      autoMode = 1;
  }
  else if(SensorValue[intAutoJumper] == 1) // Interaction Autonomous
  {
      autoMode = 2;
  }
  else if(SensorValue[bkIsoAutoJumper] == 1) // Backup Isolation Autonomous
  {
      autoMode = 3;
  }
  else if(SensorValue[isoAutoJumper] == 1) // Isolation Autonomous
  {
      autoMode = 4;
  }
}

task autonomous()
{
  switch(autoMode)
  {
      case 0:
          // No Autonomous
          break;

      case 1:
          // Programming Skills Autonomous
          sklAuto();
          break;

      case 2:
          // Interaction Autonomous
          intAuto();
          break;

      case 3:
          // Backup Isolation Autonomous
          bkIsoAuto();
          break;

      case 4:
          // Isolation Autonomous
          isoAuto();
          break;
  }

	AutonomousCodePlaceholderForTesting();  // Remove this function call once you have "real" code.
}

task usercontrol()
{
	while (true)
	{
	  if(SensorValue[singleDriverJumper] == 1)
	  {
	      oneRemoteDrive(); // One Remote
	  }
	  else
	  {
	      twoRemoteDrive(); // Two Remotes
	  }

	  UserControlCodePlaceholderForTesting(); // Remove this function call once you have "real" code.
	}
}
