/*
TITLE: 79C_IEM_AUTO_LIBRARY.H
AUTHOR: SEBASTIAN LOPEZ-COT

DESCRIPTION: THIS LIBRARY HAS SETS OF TASKS AND FUNCTIONS
             TO CONTROL THE ROBOT'S MOVEMENT USING THE
             INTEGRATED ENCODER MODULES DURING AUTONOMOUS.
*/

float wheelTicksPerRev = 392.00;
float manipTicksPerRev = 240.448;

float wheelDiameter = 4.20;
float distanceBetweenWheels = 15;

float manipGearRatio = 5.5;

float wheelCfrc = PI * wheelDiameter; // Circumference of the wheel

bool drivingFinished = true;
bool liftingFinished = true;

int leftDTEncoderLimit;
int rightDTEncoderLimit;
short lowDTSetPower;
short fullDTSetPower;
short leftMotorPower;
short rightMotorPower;
float dTSetAngleTurnAngle;
short dTSetDirection;
int dTSetDistance;

float highAngle = 440;
float medAngle = 296;
float lowAngle = 168;
float stowAngle = 0;

short manipsLiftingDirection;

int manipEncoderLimit;
short manipSetPower;
float manipSetAngle;


/* ----- ENCODER FUNCTIONS --------------------------------------------------------------------- */

void resetLeftWheelEncoders();
void resetRightWheelEncoders();
void resetBothWheelEncoders();
void resetBothManipEncoders();

void findAppropriateManipDirection(float angle);

task driveStraightForDistance();
task driveForGoal();
task pointTurn();
task swingTurn();
task liftManips();

void driveStraightForDistanceFunction(short powerLess, short powerFull, int distance) // Distance in encoder ticks
{
    if(drivingFinished == true) // Setup Code
    {
        resetBothWheelEncoders(); // Resets wheel encoders
        leftDTEncoderLimit = distance;
        rightDTEncoderLimit = distance;
        drivingFinished = false;
    }
    else
    {
        if(time1[T1] >= 40) // Update all motors at constant speed
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

		        // Stop wheels if distance is reached
		        if(abs(nMotorEncoder[leftBackWheel]) >= leftDTEncoderLimit)
		        {
		            // Stop left side
		            moveSide(0, dTSide.LEFT);
		        }

		        if(abs(nMotorEncoder[rightBackWheel]) >= rightDTEncoderLimit)
		        {
		            // Stop right side
		            moveSide(0, dTSide.RIGHT);
		        }

		        if(abs(nMotorEncoder[leftBackWheel]) >= leftDTEncoderLimit && abs(nMotorEncoder[rightBackWheel]) >= rightDTEncoderLimit)
		        {
		            // Stop both and end task
		            moveWheels(0);
		            resetBothWheelEncoders();
		            drivingFinished = true;
		            StopTask(driveStraightForDistance);
		        }

		        ClearTimer(T1); // Sets the timer back to 0
        }
    }
}

void driveForGoalFunction(short powerLess, short powerFull)
{
    resetBothWheelEncoders(); // Resets wheel encoders

    drivingFinished = false;

    while(SensorValue[goalBumpButton] == 0) // Not lined up with goal yet
    {
        if(abs(nMotorEncoder[leftBackWheel]) == abs(nMotorEncoder[rightBackWheel])) // equal distance
        {
            // Equal speed
            moveWheels(powerFull);
        }
        else if(abs(nMotorEncoder[leftBackWheel]) > abs(nMotorEncoder[rightBackWheel])) // left is greater
        {
            // Slow left side down
            moveSide(powerLess, dTSide.LEFT);
            moveSide(powerFull, dTSide.RIGHT);
        }
        else if(abs(nMotorEncoder[leftBackWheel]) < abs(nMotorEncoder[rightBackWheel])) // right is greater
        {
            // Slow right side down
            moveSide(powerFull, dTSide.LEFT);
            moveSide(powerLess, dTSide.RIGHT);
        }
    }

    // Stop driving
    moveWheels(0);
    drivingFinished = true;
}

void pointTurnFunction(short power, float angle, short direct)
{
		 // Resets encoders
		 resetBothWheelEncoders();

		 drivingFinished = false;

		 float turnCfrc = distanceBetweenWheels * PI; // Turning Circumference
		 float turnDistance = (turnCfrc / (360.00 / angle)); // Turning Distance

		 float rotations = turnDistance / wheelCfrc; // calculates number of rotations needed
		 float encoderLimitF = rotations * wheelTicksPerRev; // calculates encoder limit
		 int encoderLimit = ceil(encoderLimitF); // Rounds the encoder limit up to an integer

		if(direct == 1) // Right Turn
		{
		   //sets the motor speed
		   leftMotorPower = power;
		   rightMotorPower = power * -1;
		}
		else if(direct == 0)// Left Turn
		{
		   //sets the motor speed
		   leftMotorPower = power * -1;
		   rightMotorPower = power;
		}

		while(abs(nMotorEncoder[leftBackWheel]) < encoderLimit || abs(nMotorEncoder[rightBackWheel]) < encoderLimit)
		{
		    moveSide(leftMotorPower, dTSide.LEFT);
		    moveSide(rightMotorPower, dTSide.RIGHT);

		    if(abs(nMotorEncoder[leftBackWheel]) >= encoderLimit){ moveSide(0, dTSide.LEFT); }
		    if(abs(nMotorEncoder[rightBackWheel]) >= encoderLimit){ moveSide(0, dTSide.RIGHT); }
		}
		moveWheels(0);
		drivingFinished = true;

}

void swingTurnFunction(short power, float angle, short direction)
{
    // Does a swing turn
}

void liftManipsFunction(short power, float angle)
{
    if(liftingFinished == true) // Setup Code
    {
        if(power > 0){ manipsLiftingDirection = 1; }// Positive Voltage
        else if(power < 0){ manipsLiftingDirection = -1; }// Negative Voltage
        liftingFinished = false;
    }
    else
    {
        // Lifting/Lowering Loop
        moveManips(power);

        if(manipsLiftingDirection == 1) // Lifting
        {
            if(abs(nMotorEncoder[rightBackManip]) >= angle)
            {
                // Stop lifting
                moveManips(0);
                liftingFinished = true;
                StopTask(liftManips);
            }
        }
        else if(manipsLiftingDirection == -1) // Lowering
        {
            if(abs(nMotorEncoder[rightBackManip]) <= angle + 132) // 132 Fixes minor bug
            {
                // Stop lifting
                moveManips(0);
                liftingFinished = true;
                StopTask(liftManips);
            }
        }
    }
}


/* ----- ENCODER TASKS ------------------------------------------------------------------------- */

task driveStraightForDistance()
{
    // Acceleration Curve
    while(1 == 1)
    {
        driveStraightForDistanceFunction(lowDTSetPower, fullDTSetPower, dTSetDistance); // Subtract acceleration and deceleration(TAKE OUT MOVE(0))
    }
    // Deceleration Curve
}

task driveForGoal()
{
    // acceleration
    driveForGoalFunction(lowDTSetPower, fullDTSetPower);
}

task pointTurn()
{
    // Acceleration Curve
    pointTurnFunction(fullDTSetPower, dTSetAngleTurnAngle, dTSetDirection); // Subtract acceleration and deceleration(TAKE OUT MOVE(0))
    // Deceleration Curve
}

task swingTurn()
{
    while(1 == 1)
    {
    }
}

task liftManips()
{
    // Acceleration
    while(1 == 1)
    {
        liftManipsFunction(manipSetPower, manipSetAngle); // Subtract by acceleration and deceleration
    }
    // Deceleration
}

/* ----- ENCODER OTHER ------------------------------------------------------------------------- */

void resetLeftWheelEncoders()
{
    nMotorEncoder[leftBackWheel] = 0;
    drivingFinished = true;
}

void resetRightWheelEncoders()
{
    nMotorEncoder[rightBackWheel] = 0;
    drivingFinished = true;
}

void resetBothWheelEncoders()
{
    nMotorEncoder[leftBackWheel] = 0;
    nMotorEncoder[rightBackWheel] = 0;
    drivingFinished = true;
}

void resetBothManipEncoders()
{
    nMotorEncoder[rightBackManip] = 0;
    manipSetPower = 0;
    manipSetAngle = 0;
    liftingFinished = true;
}
