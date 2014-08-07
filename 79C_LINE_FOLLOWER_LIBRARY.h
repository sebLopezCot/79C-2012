/*
TITLE: 79C_LINE_FOLLOWER_LIBRARY.H
AUTHOR: SEBASTIAN LOPEZ-COT

DESCRIPTION: THIS LIBRARY HAS SETS OF FUNCTIONS TO
             CONTROL THE ROBOT'S MOVEMENT USING THE
             LINE SENSOR MODULES.
*/

task lineTrackForDistance();
task lineTrackForIntersection();
task lineTrackForGoal();
task driveToLine();

bool lineTrackingFinished = true;

short setLineSensorIndex;

//int lightThreshold = 1391; // Average of (average of dark values + average of light values)
int lightThreshold = 1994;

void lineTrackForDistanceFunction(short powerLess, short powerFull, int distance) // Distance in encoder ticks
{
    resetBothWheelEncoders(); // Resets wheel encoders

    lineTrackingFinished = false;

    while(abs((nMotorEncoder[leftBackWheel] + nMotorEncoder[rightBackWheel])/2) < distance)
    {
        if(SensorValue[leftLineSensor] < lightThreshold)
			  {
			      // Slow down left side
			      moveSide(powerLess, dTSide.LEFT);
			      moveSide(powerFull, dTSide.RIGHT);
			  }
			  else if(SensorValue[centerLineSensor] < lightThreshold)
			  {
			      // Drive evenly
			      moveWheels(powerFull);
			  }
			  else if(SensorValue[rightLineSensor] < lightThreshold)
			  {
			      // Slow down right side
			      moveSide(powerFull, dTSide.LEFT);
			      moveSide(powerLess, dTSide.RIGHT);
			  }
    }

    // Stop driving
    moveWheels(0);
    lineTrackingFinished = true;

}

void lineTrackForIntersectionFunction(short powerLess, short powerFull)
{
    lineTrackingFinished = false;

    while(SensorValue[leftLineSensor] >= lightThreshold || SensorValue[centerLineSensor] >= lightThreshold || SensorValue[rightLineSensor] >= lightThreshold)
    {
        // Keep Line Tracking
        if(SensorValue[leftLineSensor] < lightThreshold)
			  {
			      // Slow down left side
			      moveSide(powerLess, dTSide.LEFT);
			      moveSide(powerFull, dTSide.RIGHT);
			  }
			  else if(SensorValue[centerLineSensor] < lightThreshold)
			  {
			      // Drive evenly
			      moveWheels(powerFull);
			  }
			  else if(SensorValue[rightLineSensor] < lightThreshold)
			  {
			      // Slow down right side
			      moveSide(powerFull, dTSide.LEFT);
			      moveSide(powerLess, dTSide.RIGHT);
			  }
    }

    // Stop Driving
    moveWheels(0);
    lineTrackingFinished = true;
}

void turnOffLineFunction(short power, short direction, short sensorIndex)
{
    if(direction == 0) // Left
    {
        if(sensorIndex == 0) // Left Sensor
        {
            while(SensorValue[leftLineSensor] < lightThreshold)
            {
                moveSide(power, dTSide.LEFT);
                moveSide(power * -1, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 1) // Center Sensor
        {
            while(SensorValue[centerLineSensor] < lightThreshold)
            {
                moveSide(power, dTSide.LEFT);
                moveSide(power * -1, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 2) // Right Sensor
        {
            while(SensorValue[rightLineSensor] < lightThreshold)
            {
                moveSide(power, dTSide.LEFT);
                moveSide(power * -1, dTSide.RIGHT);
            }
            moveWheels(0);
        }
    }
    else if(direction == 1) // Right
    {
        if(sensorIndex == 0) // Left Sensor
        {
            while(SensorValue[leftLineSensor] < lightThreshold)
            {
                moveSide(power * -1, dTSide.LEFT);
                moveSide(power, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 1) // Center Sensor
        {
            while(SensorValue[centerLineSensor] < lightThreshold)
            {
                moveSide(power * -1, dTSide.LEFT);
                moveSide(power, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 2) // Right Sensor
        {
            while(SensorValue[rightLineSensor] < lightThreshold)
            {
                moveSide(power * -1, dTSide.LEFT);
                moveSide(power, dTSide.RIGHT);
            }
            moveWheels(0);
        }
    }
}

void turnOnLineFunction(short power, short direction, short sensorIndex)
{
    if(direction == 0) // Left
    {
        if(sensorIndex == 0) // Left Sensor
        {
            while(SensorValue[leftLineSensor] > lightThreshold)
            {
                moveSide(power, dTSide.LEFT);
                moveSide(power * -1, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 1) // Center Sensor
        {
            while(SensorValue[centerLineSensor] > lightThreshold)
            {
                moveSide(power, dTSide.LEFT);
                moveSide(power * -1, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 2) // Right Sensor
        {
            while(SensorValue[rightLineSensor] > lightThreshold)
            {
                moveSide(power, dTSide.LEFT);
                moveSide(power * -1, dTSide.RIGHT);
            }
            moveWheels(0);
        }
    }
    else if(direction == 1) // Right
    {
        if(sensorIndex == 0) // Left Sensor
        {
            while(SensorValue[leftLineSensor] > lightThreshold)
            {
                moveSide(power * -1, dTSide.LEFT);
                moveSide(power, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 1) // Center Sensor
        {
            while(SensorValue[centerLineSensor] > lightThreshold)
            {
                moveSide(power * -1, dTSide.LEFT);
                moveSide(power, dTSide.RIGHT);
            }
            moveWheels(0);
        }
        else if(sensorIndex == 2) // Right Sensor
        {
            while(SensorValue[rightLineSensor] > lightThreshold)
            {
                moveSide(power * -1, dTSide.LEFT);
                moveSide(power, dTSide.RIGHT);
            }
            moveWheels(0);
        }
    }
}

void lineTrackForGoalFunction(short powerLess, short powerFull)
{
    lineTrackingFinished = false;

    while(SensorValue[goalBumpButton] == 0) // Bumper not hit yet
    {
        // Keep Line Tracking
        if(SensorValue[leftLineSensor] < lightThreshold)
			  {
			      // Slow down left side
			      moveSide(powerLess, dTSide.LEFT);
			      moveSide(powerFull, dTSide.RIGHT);
			  }
			  else if(SensorValue[centerLineSensor] < lightThreshold)
			  {
			      // Drive evenly
			      moveWheels(powerFull);
			  }
			  else if(SensorValue[rightLineSensor] < lightThreshold)
			  {
			      // Slow down right side
			      moveSide(powerFull, dTSide.LEFT);
			      moveSide(powerLess, dTSide.RIGHT);
			  }
    }

    // Stop Driving
    moveWheels(0);
    lineTrackingFinished = true;
}

void driveToLineFunction(short powerLess, short powerFull, short sensorIndex)
{
    resetBothWheelEncoders(); // Resets wheel encoders

    drivingFinished = false;

    if(sensorIndex == 0) // Left
    {
        while(SensorValue[leftLineSensor] >= lightThreshold) // Not on line
        {
            // Keep wheels straight
		        if(abs(nMotorEncoder[leftBackWheel]) == abs(nMotorEncoder[rightBackWheel]))
		        {
		            // Drive equal
		            moveWheels(powerFull);
		        }
		        else if(abs(nMotorEncoder[leftBackWheel]) > abs(nMotorEncoder[rightBackWheel]))
		        {
		            // lower leftWheel power
		            moveSide(powerLess, dTSide.LEFT);
		            moveSide(powerFull, dTSide.RIGHT);
		        }
		        else if(abs(nMotorEncoder[leftBackWheel]) < abs(nMotorEncoder[rightBackWheel]))
		        {
		            // lower rightWheel power
		            moveSide(powerFull, dTSide.LEFT);
		            moveSide(powerLess, dTSide.RIGHT);
		        }
        }

        // Stop driving
        moveWheels(0);
        drivingFinished = true;
    }
    else if(sensorIndex == 1) // Center
    {
        while(SensorValue[centerLineSensor] >= lightThreshold) // Not on line
        {
            // Keep wheels straight
		        if(abs(nMotorEncoder[leftBackWheel]) == abs(nMotorEncoder[rightBackWheel]))
		        {
		            // Drive equal
		            moveWheels(powerFull);
		        }
		        else if(abs(nMotorEncoder[leftBackWheel]) > abs(nMotorEncoder[rightBackWheel]))
		        {
		            // lower leftWheel power
		            moveSide(powerLess, dTSide.LEFT);
		            moveSide(powerFull, dTSide.RIGHT);
		        }
		        else if(abs(nMotorEncoder[leftBackWheel]) < abs(nMotorEncoder[rightBackWheel]))
		        {
		            // lower rightWheel power
		            moveSide(powerFull, dTSide.LEFT);
		            moveSide(powerLess, dTSide.RIGHT);
		        }
        }

        // Stop driving
        moveWheels(0);
        drivingFinished = true;
    }
    else if(sensorIndex == 2) // Right
    {
        while(SensorValue[rightLineSensor] >= lightThreshold) // Not on line
        {
            // Keep wheels straight
		        if(abs(nMotorEncoder[leftBackWheel]) == abs(nMotorEncoder[rightBackWheel]))
		        {
		            // Drive equal
		            moveWheels(powerFull);
		        }
		        else if(abs(nMotorEncoder[leftBackWheel]) > abs(nMotorEncoder[rightBackWheel]))
		        {
		            // lower leftWheel power
		            moveSide(powerLess, dTSide.LEFT);
		            moveSide(powerFull, dTSide.RIGHT);
		        }
		        else if(abs(nMotorEncoder[leftBackWheel]) < abs(nMotorEncoder[rightBackWheel]))
		        {
		            // lower rightWheel power
		            moveSide(powerFull, dTSide.LEFT);
		            moveSide(powerLess, dTSide.RIGHT);
		        }
        }

        // Stop driving
        moveWheels(0);
        drivingFinished = true;
    }
}


/* ----- LINE FOLLOWER TASKS ------------------------------------------------------------------------- */

task lineTrackForDistance()
{
    lineTrackForDistanceFunction(lowDTSetPower, fullDTSetPower, dTSetDistance);
}

task lineTrackForIntersection()
{
    lineTrackForIntersectionFunction(lowDTSetPower, fullDTSetPower);
}

task turnOffLine()
{
    turnOffLineFunction(fullDTSetPower, dTSetDirection, setLineSensorIndex);
}

task turnOnLine()
{
    turnOnLineFunction(fullDTSetPower, dTSetDirection, setLineSensorIndex);
}

task lineTrackForGoal()
{
    lineTrackforGoalFunction(lowDTSetPower, fullDTSetPower);
}

task driveToLine()
{
    driveToLineFunction(lowDTSetPower, fullDTSetPower, setLineSensorIndex);
}
