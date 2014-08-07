/*
TITLE: 79C_GENERAL_COMMANDS.H
AUTHOR: SEBASTIAN LOPEZ-COT

DESCRIPTION: GENERAL COMMANDS USED IN OTHER PARTS OF THE CODE.
*/

typedef struct
{
    short LEFT;
    short RIGHT;
}Side;

typedef struct
{
    short LEFT;
    short RIGHT;
}Direction;

typedef struct
{
    short LEFT;
    short CENTER;
    short RIGHT;
}LineFollowerSet;

Side dTSide; // Drivetrain Side Struct
Direction direction; // Direction Struct
LineFollowerSet dTLineFollowers; // Drivetrain Line Follower Struct

void initTypedefs()
{
    // Drivetrain Side
    dTSide.LEFT = 0;
    dTSide.RIGHT = 1;

    // Direction
    direction.LEFT = 0;
    direction.RIGHT = 1;

    // Line Followers
    dTLineFollowers.LEFT = 0;
    dTLineFollowers.CENTER = 1;
    dTLineFollowers.RIGHT = 2;
}

void overrideDirection()
{
    direction.LEFT = 1;
    direction.RIGHT = 0;
}

void moveWheels(short power)
{
    motor[leftBackWheel] = power;
    motor[rightBackWheel] = power;
    motor[leftFrontWheel] = power;
    motor[rightFrontWheel] = power;
}

void moveSide(short power, short side)
{
    if(side == 0) // Left Side
    {
        motor[leftBackWheel] = power;
        motor[leftFrontWheel] = power;
    }
    else if(side == 1) // Right Side
    {
        motor[rightBackWheel] = power;
        motor[rightFrontWheel] = power;
    }
}

void moveManips(short power)
{
    motor[leftFrontManip] = power;
    motor[leftBackManip] = power;
    motor[rightFrontManip] = power;
    motor[rightBackManip] = power;
}

void moveSweepers(short power)
{
    motor[leftSweeper] = power;
    motor[rightSweeper] = power;
}

void shootPieces(short power, int timeInMills)
{
    moveSweepers(power);
    wait1Msec(timeInMills);
    moveSweepers(0);
}

int reverseSign(int number)
{
    if(number != 0)
    {
        return number * -1;
    }
    else
    {
        return 0;
    }

}
