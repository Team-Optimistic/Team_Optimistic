#ifndef TURNINGFUNCTIONS_C_INCLUDED
#define TURNINGFUNCTIONS_C_INCLUDED

/**
 * Turns clockwise for an angle in degrees
 * @param  angle Angle to turn for (deg)
 */
void turn(const long angle)
{
	//Save left and right quad values instead of setting them to zero
	long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total angle change since start
	float angleChange = 0, lastAngle = 0;

	//Conversion between encoder degrees and base_link degrees
	const float conv = 12.75993;

	//Target angle
	int targetAngle = angle * conv;

	pos_PID anglePID;
	if (fabs(angle) <= 350)
	{
		pos_PID_InitController(&anglePID, &angleChange, 0.6, 0.45, 0.1);
	}
	else
	{
		pos_PID_InitController(&anglePID, &angleChange, 0.6, 0.45, 0.2);
	}
	pos_PID_SetTargetPosition(&anglePID, targetAngle);

	//If angle PID controller is at target
	bool atTarget = false;

	//Angle that is "close enough" to target
	const int atTargetAngle = 10;

	//Threshold for not moving
	const int threshold = 2;

	//Timer for being at target
	timer atTargetTimer;
	timer_Initialize(&atTargetTimer);

	//Timeout period (ms)
	const int timeoutPeriod = 250;

	//Current left and right quad displacements
	long currentLeft, currentRight;

	//Distance and angle PID output
	int angleOutput;

	while (!atTarget)
	{
		//Calculate distance displacement
		currentLeft = SensorValue[leftQuad] - encoderLeft;
		currentRight = SensorValue[rightQuad] - encoderRight;

		angleChange = currentLeft - currentRight;

		//Get output from PID
		angleOutput = pos_PID_StepController(&anglePID);

		//Set motors to angle PID output
		setLeftMotors(angleOutput);
		setRightMotors(-1 * angleOutput);

		//Place mark if we're close enough to the target angle
		if (fabs(targetAngle - angleChange) <= atTargetAngle)
		{
			timer_PlaceHardMarker(&atTargetTimer);
		}
		//Place mark if we haven't moved much
		else if (fabs(angleChange - lastAngle) <= threshold)
		{
			timer_PlaceHardMarker(&atTargetTimer);
		}
		else
		{
			timer_ClearHardMarker(&atTargetTimer);
		}

		lastAngle = angleChange;

		//If we've been close enough for long enough, we're there
		if (timer_GetDTFromHardMarker(&atTargetTimer) >= timeoutPeriod)
		{
			atTarget = true;
		}

		wait1Msec(15);
	}

	setAllDriveMotors(0);
}

/**
 * Turns to an angle in the field frame
 * @param deg Degrees to turn to
 */
void turnToAbsAngle(const long deg)
{
	long theta = 0;
	BCI_lockSem(std_msgSem, "turnToAbsAngle")
	{
		theta = std_msg[STD_MSG_EST_THETA];
		BCI_unlockSem(std_msgSem, "turnToAbsAngle")
	}

	const long turnAmt = deg - theta;

	if (turnAmt > 180)
	{
		turn(360 - turnAmt);
	}
	else if (turnAmt < -180)
	{
		turn(360 + turnAmt);
	}
	else
	{
		turn(turnAmt);
	}
}

#endif //TURNINGFUNCTIONS_C_INCLUDED
