/*
TITLE: 79C_DRIVER_CONFIG.H
AUTHOR: SEBASTIAN LOPEZ-COT

DESCRIPTION: CONFIGURATION FOR DRIVER CONTROL MODE. HANDLES SWITCHING
             BETWEEN TWO REMOTES AND ONE REMOTE.
*/

bool driverPID = false;
bool speedLowered = false;
short reCalibrateButtonTimer = 0;

float controllerExp = 2.3;

void oneRemoteDrive()
{
    // Drivetrain (Arcade)
    if(!autoDrivingToGoal && !speedLowered)
    {
        moveSide(vexRT[Ch3] + vexRT[Ch4], dTSide.LEFT);
        moveSide(vexRT[Ch3] - vexRT[Ch4], dTSide.RIGHT);
    }
    else if(!autoDrivingToGoal && speedLowered)
    {
        moveSide((vexRT[Ch3]/2) + (vexRT[Ch4]/2), dTSide.LEFT);
        moveSide((vexRT[Ch3]/2) - (vexRT[Ch4]/2), dTSide.RIGHT);
    }
    else
    {
        // Auto backup button
        autoBackFromGoal(-44, -69, dTSetDistance);
    }

    // Goal Back up Buttons
    if(vexRT[Btn8D] == 1 && SensorValue[goalBumpButton] == 1 && abs(nMotorEncoder[rightBackManip]) >= lowAngle) // 10"
    {
        dTSetDistance = 300;
        autoDrivingToGoal = true;
        resetBothWheelEncoders();
    }
    else if(vexRT[Btn8R] == 1 && SensorValue[goalBumpButton] == 1 && abs(nMotorEncoder[rightBackManip]) >= lowAngle) // 20"
    {
        dTSetDistance = 300;
        autoDrivingToGoal = true;
        resetBothWheelEncoders();
    }
    else if(vexRT[Btn8D] == 1 && abs(nMotorEncoder[rightBackManip]) >= lowAngle) // 10" and Speed Lowered
    {
        speedLowered = true;
    }
    else if(vexRT[Btn8R] == 1 && abs(nMotorEncoder[rightBackManip]) >= medAngle)// 20" and Speed Lowered
    {
        speedLowered = true;
    }
    else if(vexRT[Btn8U] == 1 && abs(nMotorEncoder[rightBackManip]) >= highAngle) // 30" and Speed Lowered
    {
        speedLowered = true;
    }
    else if(vexRT[Btn8D] == 0 && vexRT[Btn8R] == 0 && vexRT[Btn8U] == 0) // Speed Full
    {
        speedLowered = false;
    }

    if(vexRT[Btn8L] == 1) // Break
    {
        autoDrivingToGoal = false;
    }

    // Manipulator
    if(!manipsLifting)
    {
        moveManips(vexRT[Ch2]);
    }
    else if(manipsLifting)
    {
        liftManipsToGoal(manipSetPower, goalAngle);
    }

    // Manipulator Buttons
    if(vexRT[Btn7U] == 1 && reCalibrateButtonTimer < 15) // Calibrate Manips
    {
        reCalibrateButtonTimer += 1; // Increment Counter
    }
    else if(vexRT[Btn7U] == 1 && reCalibrateButtonTimer >= 15)
    {
        liftingFinished = true;
        manipsLifting = false;
        resetBothManipEncoders(); // Reset Encoders
        reCalibrateButtonTimer = 0;

        // Lift to 11" Height for feedback
        liftingFinished = true;
        manipsLifting = true;
        goalAngle = lowAngle;
        findAppropriateManipDirection(lowAngle); // Determine need to go up or down
    }
    else
    {
        reCalibrateButtonTimer = 0;
    }

    if(vexRT[Btn8U] == 1) // 30"
    {
        manipsLifting = true;
        goalAngle = highAngle;
        findAppropriateManipDirection(highAngle); // Determine need to go up or down
    }
    else if(vexRT[Btn8R] == 1) // 20"
    {
        manipsLifting = true;
        goalAngle = medAngle;
        findAppropriateManipDirection(medAngle); // Determine need to go up or down
    }
    else if(vexRT[Btn8D] == 1) // 11"
    {
        manipsLifting = true;
        goalAngle = lowAngle;
        findAppropriateManipDirection(lowAngle); // Determine need to go up or down
    }


    if(vexRT[Btn8L] == 1) // Break
    {
        liftingFinished = true;
        manipsLifting = false;
    }

    // Sweepers (Negative is out)
    if(vexRT[Btn6D] == 1){ moveSweepers(127); } // Pull in

    else if(vexRT[Btn6U] == 1){ moveSweepers(-127); } // Shoot out

    else if(vexRT[Btn6U] == 1 && vexRT[Btn6D] == 1){ moveSweepers(0); } // Stop

    else if(vexRT[Btn6U] == 0 && vexRT[Btn6D] == 0){ moveSweepers(0); } // Stop
}

void twoRemoteDrive()
{
    // Drivetrain (Tank)
    if(!driverPID && !autoDrivingToGoal && !speedLowered)
    {
				double y1 = pow(abs(vexRT[Ch3]), controllerExp) / 16129;
				double y2 = pow(abs(vexRT[Ch2]), controllerExp) / 16129;
		    if(vexRT[Ch3] >= 0)
		    {
		        moveSide(127 * y1, dTSide.LEFT);
		    }
		    else if(vexRT[Ch3] < 0)
		    {
		        moveSide(-127 * y1, dTSide.LEFT);
		    }

		    if(vexRT[Ch2] >= 0)
		    {
		        moveSide(127 * y2, dTSide.RIGHT);
		    }
		    else if(vexRT[Ch2] < 0)
		    {
		        moveSide(-127 * y2, dTSide.RIGHT);
		    }
    }
    else if(!driverPID && !autoDrivingToGoal && speedLowered)
    {
        moveSide(vexRT[Ch3]/2, dTSide.LEFT);
        moveSide(vexRT[Ch2]/2, dTSide.RIGHT);
    }
    else if(driverPID && !autoDrivingToGoal && !speedLowered)
    {
        moveSidePID(vexRT[Ch3], dTSide.LEFT);
        moveSidePID(vexRT[Ch2], dTSide.RIGHT);
    }
    else if(driverPID && !autoDrivingToGoal && speedLowered)
    {
        moveSidePID(vexRT[Ch3]/2, dTSide.LEFT);
        moveSidePID(vexRT[Ch2]/2, dTSide.RIGHT);
    }
    else if(autoDrivingToGoal)
    {
        // Auto backup button
        autoBackFromGoal(-87, -127, dTSetDistance);
    }

    // PID Switching
    if(vexRT[Btn6U] == 1){ driverPID = false; }

    else if(vexRT[Btn6D] == 1){ driverPID = true; }

    // Goal Back up Buttons
    if(vexRT[Btn8D] == 1 && SensorValue[goalBumpButton] == 1) // 10"
    {
        dTSetDistance = 300;
        autoDrivingToGoal = true;
        resetBothWheelEncoders();
    }
    else if(vexRT[Btn8R] == 1 && SensorValue[goalBumpButton] == 1) // 20"
    {
        dTSetDistance = 300;
        autoDrivingToGoal = true;
        resetBothWheelEncoders();
    }
    else if(vexRT[Btn8D] == 1 && abs(nMotorEncoder[rightBackManip]) >= lowAngle) // 10" and Speed Lowered
    {
        speedLowered = true;
    }
    else if(vexRT[Btn8R] == 1 && abs(nMotorEncoder[rightBackManip]) >= medAngle)// 20" and Speed Lowered
    {
        speedLowered = true;
    }
    else if(vexRT[Btn8U] == 1 && abs(nMotorEncoder[rightBackManip]) >= highAngle) // 30" and Speed Lowered
    {
        speedLowered = true;
    }
    else if(vexRT[Btn8D] == 0 && vexRT[Btn8R] == 0 && vexRT[Btn8U] == 0) // Speed Full
    {
        speedLowered = false;
    }

    if(vexRT[Btn8L] == 1) // Break
    {
        autoDrivingToGoal = false;
    }


    // Manipulator
    if(!manipsLifting)
    {
        double y1 = pow(abs(vexRT[Ch2Xmtr2]), controllerExp) / 16129;
        if(vexRT[Ch2Xmtr2] >= 0 && nMotorEncoder[rightBackManip] < highAngle - 100)
        {
           moveManips(127 * y1);
        }
		    else if(vexRT[Ch2Xmtr2] >= 0 && nMotorEncoder[rightBackManip] >= highAngle - 100)
		    {
		        moveManips((127 * y1)/2); // MAYBE NOT 2
		    }
		    else if(vexRT[Ch2Xmtr2] < 0)
		    {
		        moveManips(-127 * y1);
		    }
    }
    else if(manipsLifting)
    {
        liftManipsToGoal(manipSetPower, goalAngle);
    }

    // Manipulator Buttons
    if(vexRT[Btn7UXmtr2] == 1 && reCalibrateButtonTimer < 15) // Calibrate Manips
    {
        reCalibrateButtonTimer += 1; // Increment Counter
    }
    else if(vexRT[Btn7UXmtr2] == 1 && reCalibrateButtonTimer >= 15)
    {
        liftingFinished = true;
        manipsLifting = false;
        resetBothManipEncoders(); // Reset Encoders
        reCalibrateButtonTimer = 0;

        // Lift to 11" Height for feedback
        liftingFinished = true;
        manipsLifting = true;
        goalAngle = lowAngle;
        findAppropriateManipDirection(lowAngle); // Determine need to go up or down
    }
    else
    {
        reCalibrateButtonTimer = 0;
    }

    if(vexRT[Btn8UXmtr2] == 1) // 30"
    {
        manipsLifting = true;
        goalAngle = highAngle;
        findAppropriateManipDirection(highAngle); // Determine need to go up or down
    }
    else if(vexRT[Btn8RXmtr2] == 1) // 20"
    {
        manipsLifting = true;
        goalAngle = medAngle;
        findAppropriateManipDirection(medAngle); // Determine need to go up or down
    }
    else if(vexRT[Btn8DXmtr2] == 1) // 11"
    {
        manipsLifting = true;
        goalAngle = lowAngle;
        findAppropriateManipDirection(lowAngle); // Determine need to go up or down
    }


    if(vexRT[Btn8LXmtr2] == 1) // Break
    {
        liftingFinished = true;
        manipsLifting = false;
    }


    // Sweepers (Negative is out)
    if(vexRT[Btn6DXmtr2] == 1){ moveSweepers(127); } // Pull in

    else if(vexRT[Btn6UXmtr2] == 1){ moveSweepers(-127); } // Shoot out

    else if(vexRT[Btn6UXmtr2] == 1 && vexRT[Btn6DXmtr2] == 1){ moveSweepers(0); } // Stop

    else if(vexRT[Btn6UXmtr2] == 0 && vexRT[Btn6DXmtr2] == 0){ moveSweepers(0); } // Stop
}
