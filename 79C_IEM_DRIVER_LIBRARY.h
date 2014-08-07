/*
TITLE: 79C_IEM_DRIVER_LIBRARY.H
AUTHOR: SEBASTIAN LOPEZ-COT

DESCRIPTION: THIS LIBRARY HAS SETS OF FUNCTIONS TO
             CONTROL THE ROBOT'S MOVEMENT USING THE
             INTEGRATED ENCODER MODULES DURING AUTONOMOUS.
*/

bool manipsLifting = false;
bool autoDrivingToGoal = false;

int remainingEncoderTicks;

short treshold = 40;
short interval = 75;

float goalAngle;

void moveSidePID(short power, short side)
{
    // Moves side of drivetrain in a smooth speed curve

    if(side == 0) // Left
    {
        if(time1[T1] >= 40) // 40ms have passed since last motor update
        {
           short speedAverage = (motor[leftFrontWheel] + motor[leftBackWheel]) / 2; // Average of both motor speeds
           int range = abs(power - speedAverage); // range between goal speed and actual speed

           if(range <= treshold) // range is less than or equal to threshold
           {
              moveSide(power, dTSide.LEFT); // move left side at goal power
           }
           else // range is greater than treshold
           {
              if((power - speedAverage) < 0) // negative voltage
              {
                  moveSide(speedAverage - interval, dTSide.LEFT); // subtract until reaches threshold
              }
              else if((power - speedAverage) > 0) // positive voltage
              {
                  moveSide(speedAverage + interval, dTSide.LEFT); // add until reaches threshold
              }
           }

           ClearTimer(T1); // Resets timer to 0 seconds
        }
    }
    else if(side == 1) // Right
    {
        if(time1[T2] >= 40) // 40ms have passed since last motor update
        {
           short speedAverage = (motor[rightFrontWheel] + motor[rightBackWheel]) / 2; // Average of both motor speeds
           int range = abs(power - speedAverage); // range between goal speed and actual speed

           if(range <= treshold) // range is less than or equal to threshold
           {
              moveSide(power, dTSide.RIGHT); // move right side at goal power
           }
           else // range is greater than treshold
           {
              if((power - speedAverage) < 0) // negative voltage
              {
                  moveSide(speedAverage - interval, dTSide.RIGHT); // subtract until reaches threshold
              }
              else if((power - speedAverage) > 0) // positive voltage
              {
                  moveSide(speedAverage + interval, dTSide.RIGHT); // add until reaches threshold
              }
           }

           ClearTimer(T2); // Resets timer to 0 seconds
        }
    }
}

void autoBackFromGoal(short powerLess, short powerFull, int distanceInEncoderTicks) // Function for driving back from the goal
{
    if((abs(nMotorEncoder[leftBackWheel]) + abs(nMotorEncoder[rightBackWheel]))/2 < distanceInEncoderTicks) // Not there yet
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
    else // There now
    {
        // Stop
        moveWheels(0);
        autoDrivingToGoal = false;
    }
}

void liftManipsToGoal(short power, float angle)
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
                manipsLifting = false;
                liftingFinished = true;
            }
        }
        else if(manipsLiftingDirection == -1) // Lowering
        {
            if(abs(nMotorEncoder[rightBackManip]) <= angle + 132) // 132 Fixes minor bug
            {
                // Stop lifting
                moveManips(0);
                manipsLifting = false;
                liftingFinished = true;
            }
        }
    }
}

void findAppropriateManipDirection(float angle)
{
    if(abs(nMotorEncoder[rightBackManip]) < angle) // Lower than Goal Angle
    {
        manipSetPower = 127; // Lift Manipulator
    }
    else if(abs(nMotorEncoder[rightBackManip]) > angle) // Higher than Goal Angle
    {
        manipSetPower = -127; // Lower Manipulator
    }
    else if(abs(nMotorEncoder[rightBackManip]) == angle) // Already at Goal Angle
    {
        liftingFinished = true;
        manipsLifting = false;
    }
}
