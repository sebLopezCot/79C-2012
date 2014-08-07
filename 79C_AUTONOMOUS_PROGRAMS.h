/*
TITLE: 79C_AUTONOMOUS_PROGRAMS.H
AUTHOR: SEBASTIAN LOPEZ-COT

DESCRIPTION: THIS LIBRARY CONTAINS THE AUTONMOUS PROGRAMS
THAT ARE USEABLE ON THE ROBOT.
*/

void startDrivingStraightDistance(short powerLess, short powerFull, int distanceInEncoderTicks);
void startDrivingForGoal(short powerLess, short powerFull);
void startPointTurning(short power, float angle, short direction);
void startLiftingManips(  float angle);
void startLineTracking(short powerLess, short powerFull, int distanceInEncoderTicks);
void startLineTrackingForIntersection(short powerLess, short powerFull);
void startTurningOffLine(short power, short direction, short sensorIndex);
void startTurningOnLine(short power, short direction, short sensorIndex);
void startLineTrackingForGoal(short powerLess, short powerFull);
void startDrivingToLine(short powerLess, short powerFull, short sensorIndex);
void smoothAccel200ms(short power);
void waitForButtonPress();
void stowManips();
task encoderResetLoop();

short endNumber = 0;
int distanceTwenty = 0;

void isoAuto() // Isolation Autonomous
{
  // Pick up barrel
  resetBothWheelEncoders();
  int changeSpeed = 1;
  moveSweepers(127);
  while(abs(nMotorEncoder[leftBackWheel]) < 300)
  {
    if(changeSpeed < 127){ changeSpeed += 1; }
    moveWheels(changeSpeed);
  }
  moveWheels(0);

  wait1Msec(1000);

  moveSweepers(0);

  // Pick up ball
  resetBothWheelEncoders();
  changeSpeed = 1;
  moveSweepers(127);
  while(abs(nMotorEncoder[leftBackWheel]) < 180)
  {
    if(changeSpeed < 127){ changeSpeed += 1; }
    moveWheels(changeSpeed);
  }
  moveWheels(0);

  wait1Msec(1000);

  moveSweepers(0);

  // Pick up barrel
  resetBothWheelEncoders();
  changeSpeed = 1;
  moveSweepers(127);
  while(abs(nMotorEncoder[leftBackWheel]) < 180)
  {
    if(changeSpeed < 127){ changeSpeed += 1; }
    moveWheels(changeSpeed);
  }
  moveWheels(0);

  wait1Msec(1250);

  moveSweepers(0);

  // Move back to tile
  resetBothWheelEncoders();
  changeSpeed = -1;
  while(abs(nMotorEncoder[leftBackWheel]) < 940)
  {
    if(changeSpeed > -127){ changeSpeed -= 1; }
    moveWheels(changeSpeed);
  }
  moveWheels(0);

  // Lift to 30" Goal
  startLiftingManips(highAngle);

  while(liftingFinished == false){}

  // Wait for User
  waitForButtonPress();

  wait1Msec(1000);

  if(SensorValue[autoProgButton] == 1){ endNumber = 1; }

  if(endNumber == 0) // 30" Goal
  {
	    // Lift to 30" Goal
	    startLiftingManips(highAngle + 20);

	    while(liftingFinished == false){}

	    // Shoot Pieces
	    shootPieces(-127, 4000);

	    waitForButtonPress();

	    // Lower Manipulator
	    stowManips();

	    // Wait for User
	    waitForButtonPress();

	    // Get ball
	    resetBothWheelEncoders();
	    changeSpeed = 1;
	    moveSweepers(127);
	    while(abs(nMotorEncoder[leftBackWheel]) < 535)
	    {
	      if(changeSpeed < 127){ changeSpeed += 1; }
	      moveWheels(changeSpeed);
	    }
	    moveWheels(0);

	    wait1Msec(1000);

	    // Point turn
	    startPointTurning(127, 45, direction.LEFT);

	    while(drivingFinished == false){}

	    // Get barrel
	    resetBothWheelEncoders();
	    changeSpeed = 1;
	    while(abs(nMotorEncoder[leftBackWheel]) < 200)
	    {
	      if(changeSpeed < 127){ changeSpeed += 1; }
	      moveWheels(changeSpeed);
	    }
	    moveWheels(0);

	    wait1Msec(1000);

	    moveSweepers(0);

	    // Move back
	    moveWheels(-127);

	    wait1Msec(800);

	    moveWheels(0);

	    // Get ready for driver
	    startPointTurning(95, 67, direction.RIGHT);
	    startLiftingManips(medAngle);
  }
  else if(endNumber == 1) // 20" Goal
  {
      resetBothWheelEncoders();
      // smoothAccel200ms(127);
      // while(abs(nMotorEncoder[leftBackWheel]) < 360){}
      // startLiftingManips(medAngle);
      // while(liftingFinished == false){}
      // startLiftingManips(medAngle);
      // while(liftingFinished == false){}
      // startPointTurning(95, 32, direction.LEFT);
      // while(drivingFinished == false){}
      smoothAccel200ms(127);
      while(abs(nMotorEncoder[leftBackWheel]) < 360){}
      startPointTurning(95, 32, direction.RIGHT);
      while(drivingFinished == false){}
      wait1Msec(250);
      startDrivingStraightDistance(40, 80, 360);
      while(drivingFinished == false){}
      startPointTurning(95, 67, direction.LEFT);
      while(drivingFinished == false){}
      startDrivingStraightDistance(40, 80, 400);
      startLiftingManips(medAngle);
      while(liftingFinished == false || drivingFinished == false){}
      shootPieces(-127, 1300);
      startDrivingStraightDistance(-30, -80, 240);
      while(drivingFinished == false){}
      shootPieces(-127, 1300);
      wait1Msec(500);
      // startPointTurning(95, 32, direction.LEFT);
      // stowManips();
      // while(drivingFinished == true){}
      // moveSweepers(127);
      // startDrivingStraightDistance(30, 127, 400);
      // while(drivingFinished == false){}
      // wait1Msec(1000);
      // moveSweepers(0);
  }

}


void bkIsoAuto() // Backup Isolation Autonomous
{
    waitForButtonPress();

    startDrivingToLine(40, 127, dTLineFollowers.CENTER);
    while(drivingFinished == false){}

}

void intAuto() // Interaction Autonomous
{
  // Knock stack
  moveWheels(127);
  wait1Msec(2000);
  moveWheels(-127);
  wait1Msec(2000);

  // Load pieces
  waitForButtonPress();

  // Lift to 10" height
  startLiftingManips(lowAngle);
  while(liftingFinished == false){}
  wait1Msec(500);
  startLiftingManips(lowAngle); // Second check
  while(liftingFinished == false){}

  waitForButtonPress();

  // Shoot pieces
  shootPieces(-127, 1500);

  waitForButtonPress();

  // Lower Arm
  stowManips();

  waitForButtonPress();
  wait1Msec(1000);
  if(SensorValue[autoProgButton] == 1)
  {
      endNumber = 1;
      startLiftingManips(lowAngle);
      wait1Msec(1000);
      if(SensorValue[autoProgButton] == 1){ endNumber = 2; }
  }

  if(endNumber == 0) // Shoot in 30" Goal
  {
    startLiftingManips(highAngle); // Lift arm
    smoothAccel200ms(127);
    startDrivingForGoal(40, 127); // Drive to goal
    while(drivingFinished == false || liftingFinished == false){}
    startLiftingManips(highAngle);
    while(liftingFinished == false){} // Double check
    shootPieces(-127, 4000);
  }
  else if(endNumber == 1) // Pass pieces into isolation
  {
    startLiftingManips(medAngle);
    smoothAccel200ms(127);
    wait1Msec(2000);
    moveWheels(0);
    shootPieces(-127, 5000);
    moveWheels(-127);
    wait1Msec(2000);
    moveWheels(0);
    stowManips();
  }
  else if(endNumber == 2) // Pick up pieces on the floor
  {
    stowManips();
    waitForButtonPress();
    moveSweepers(127);
    smoothAccel200ms(127);
    startDrivingStraightDistance(30, 127, 560);
    while(drivingFinished == false){}
    wait1Msec(1000);
    moveSweepers(0);
  }
}

void sklAuto() // Programming Skills Autonomous
{

  // Pick up stack in front of tile
  moveSweepers(127);
  startDrivingStraightDistance(107, 127, 240);
  while(drivingFinished == false){}
  wait1Msec(1000);
  moveSweepers(0);

  // Load in ball
  startDrivingStraightDistance(-107, -127, 240);

  waitForButtonPress();

  // Pick up doubler
  startDrivingStraightDistance(30, 127, 2550);
  while(abs(nMotorEncoder[leftBackWheel]) < 1070){}
  moveSweepers(127);
  while(drivingFinished == false){}
  wait1Msec(2000);
  moveSweepers(0);

  // Turn to Goal
  startPointTurning(64, 75, direction.RIGHT);
  while(drivingFinished == false){}

  // Pick up 30" stack
  moveSweepers(127);
  startDrivingStraightDistance(32, 64, 357);
  while(drivingFinished == false){}
  wait1Msec(800); // WAS 1000 MS
  moveSweepers(0);

  // Score in 30" Goal
  startDrivingStraightDistance(-80, -100, 120);
  startLiftingManips(highAngle);
  while(drivingFinished == false || liftingFinished == false){}
  startDrivingForGoal(40, 80); // WAS 20 AND 40
  while(drivingFinished == false){}
  startLiftingManips(highAngle);
  while(liftingFinished == false){}
  startDrivingStraightDistance(-80, -100, 80);
  while(drivingFinished == false){}
  shootPieces(-127, 5000);

  // Point turn to 20" goal
  // startDrivingStraightDistance(-80, -100, 240); // WAS 357
  // while(abs(nMotorEncoder[leftBackWheel]) < 178){}
  // stowManips();
  // while(drivingFinished == false){}
  // startPointTurning(64, 150, direction.RIGHT); // WAS 180
  // while(drivingFinished == false){}

  //waitForButtonPress();

  // Pick up 20" stack
  // moveSweepers(127);
  // startDrivingStraightDistance(32, 64, 357);
  // while(drivingFinished == false){}
  // wait1Msec(800); // WAS 1000
  // moveSweepers(0);

  //waitForButtonPress();

  // Score in 20" goal
  // startDrivingStraightDistance(-80, -100, 120);
  // startLiftingManips(medAngle);
  // while(drivingFinished == false || liftingFinished == false){}
  // startDrivingForGoal(40, 80);
  // while(drivingFinished == false){}
  // shootPieces(-127, 800);
  // startDrivingStraightDistance(-80, -100, 180);
  // while(drivingFinished == false){}
  // shootPieces(-127, 800);

  // waitForButtonPress();

  // Back to tile
  moveSide(-127, direction.LEFT);
  wait1Msec(900);
  startDrivingStraightDistance(-30, -127, 2550);
  stowManips();
  while(drivingFinished == false){}
  startLiftingManips(lowAngle);

  waitForButtonPress();

  // Point turn to 30" goal
  // startPointTurning(64, 25, direction.LEFT);
  // while(drivingFinished == false){}
  // startLiftingManips(highAngle);
  // while(liftingFinished == false){}

  // Drive to 30" goal
  // moveWheels(32);
  // wait1Msec(250);
  // moveWheels(64);
  // wait1Msec(250);
  // moveWheels(95);
  // wait1Msec(250);
  // moveWheels(127);
  // wait1Msec(250);

  // startDrivingForGoal(30, 127);
  // while(drivingFinished == false){}
  // startLiftingManips(highAngle);
  // while(liftingFinished == false){}
  // shootPieces(-127, 4250);

  // Go for 10" Goal
  startDrivingForGoal(40, 80);
  while(drivingFinished == false){}
  startDrivingStraightDistance(-80, -100, 300);
  while(drivingFinished == false){}
  shootPieces(-127, 1000);
  startDrivingStraightDistance(-30, -127, 300);
  stowManips();

  waitForButtonPress();

  // send to other tile
  startDrivingStraightDistance(30, 127, 2000);
  while(drivingFinished == false){}
  startLiftingManips(lowAngle);

  waitForButtonPress();

  // shoot in 10" Goal
  startDrivingForGoal(40, 80);
  while(drivingFinished == false){}
  startDrivingStraightDistance(-80, -100, 240);
  while(drivingFinished == false){}
  shootPieces(-127, 1000);
  startDrivingStraightDistance(-30, -127, 240);
  stowManips();

  // Pick up stack in front of tile
  waitForButtonPress();
  moveSweepers(127);
  startDrivingStraightDistance(107, 127, 240);
  while(drivingFinished == false){}
  wait1Msec(1000);
  moveSweepers(0);

  // Load in ball
  startDrivingStraightDistance(-107, -127, 240);

  waitForButtonPress();

  // Pick up doubler
  startDrivingStraightDistance(30, 127, 2550);
  while(abs(nMotorEncoder[leftBackWheel]) < 1070){}
  moveSweepers(127);
  while(drivingFinished == false){}
  wait1Msec(2000);
  moveSweepers(0);

  // Turn to Goal
  startPointTurning(64, 75, direction.LEFT);
  while(drivingFinished == false){}

  // Pick up 30" stack
  moveSweepers(127);
  startDrivingStraightDistance(32, 64, 357);
  while(drivingFinished == false){}
  wait1Msec(800); // WAS 1000 MS
  moveSweepers(0);

  // Score in 30" Goal
  startDrivingStraightDistance(-80, -100, 120);
  startLiftingManips(highAngle);
  while(drivingFinished == false || liftingFinished == false){}
  startDrivingForGoal(40, 80); // WAS 20 AND 40
  while(drivingFinished == false){}
  startLiftingManips(highAngle);
  while(liftingFinished == false){}
  startDrivingStraightDistance(-80, -100, 80);
  while(drivingFinished == false){}
  shootPieces(-127, 5000);
}


/* ----- EASY TASK STARTING FUNCTIONS ----------------------------------------------------- */

void startDrivingStraightDistance(short powerLess, short powerFull, int distanceInEncoderTicks)
{
  lowDTSetPower = powerLess;
  fullDTSetPower = powerFull;
  dTSetDistance = distanceInEncoderTicks;
  StartTask(driveStraightForDistance);
}

void startDrivingForGoal(short powerLess, short powerFull)
{
  lowDTSetPower = powerLess;
  fullDTSetPower = powerFull;
  StartTask(driveForGoal);
}

void startPointTurning(short power, float angle, short direction)
{
  fullDTSetPower = power;
  dTSetAngleTurnAngle = angle;
  dTSetDirection = direction;
  StartTask(pointTurn);
}

void startLiftingManips(float angle)
{
  findAppropriateManipDirection(angle); // Determines whether to lift or lower
  manipSetAngle = angle;
  liftingFinished = true;
  StartTask(liftManips);
}

void startLineTracking(short powerLess, short powerFull, int distanceInEncoderTicks)
{
  lowDTSetPower = powerLess;
  fullDTSetPower = powerFull;
  dTSetDistance = distanceInEncoderTicks;
  StartTask(lineTrackForDistance);
}

void startLineTrackingForIntersection(short powerLess, short powerFull)
{
  lowDTSetPower = powerLess;
  fullDTSetPower = powerFull;
  StartTask(lineTrackForIntersection);
}

void startTurningOffLine(short power, short direction, short sensorIndex)
{
  fullDTSetPower = power;
  dTSetDirection = direction;
  setLineSensorIndex = sensorIndex;
  StartTask(turnOffLine);
}
void startTurningOnLine(short power, short direction, short sensorIndex)
{
  fullDTSetPower = power;
  dTSetDirection = direction;
  setLineSensorIndex = sensorIndex;
  StartTask(turnOnLine);
}

void startLineTrackingForGoal(short powerLess, short powerFull)
{
  lowDTSetPower = powerLess;
  fullDTSetPower = powerFull;
  StartTask(lineTrackForGoal);
}

void startDrivingToLine(short powerLess, short powerFull, short sensorIndex)
{
  lowDTSetPower = powerLess;
  fullDTSetPower = powerFull;
  setLineSensorIndex = sensorIndex;
  StartTask(driveToLine);
}

void smoothAccel200ms(short power)
{
    moveWheels(power * .2);
    wait1Msec(100);
    moveWheels(power * .6);
    wait1Msec(100);
    moveWheels(power);
}

void waitForButtonPress()
{
  while(SensorValue[autoProgButton] == 0){}
}

void stowManips()
{
  while(SensorValue[stowAngleButton] == 0)
  {
      moveManips(-127);
  }
  moveManips(0);
}

task encoderResetLoop()
{
  while(1 == 1)
  {
    if(SensorValue[stowAngleButton] == 1)
    {
      resetBothManipEncoders();
    }
  }
}
