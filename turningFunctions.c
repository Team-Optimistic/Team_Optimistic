#ifndef TURNINGFUNCTIONS_C_INCLUDED
#define TURNINGFUNCTIONS_C_INCLUDED

/**
 * Turns clockwise for an angle in degrees
 * @param  angle Angle to turn for (deg)
 */
void turn(long angle)
{
	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = nMotorEncoder[driveLFY], encoderRight = nMotorEncoder[driveRFY];

	//Total angle change since start
	float angleChange = 0, lastAngle = 0;

	//Conversion between encoder degrees and base_link degrees
	const float conv = 12.88361;

	//Fix angle
	while(angle>180)
      angle-=360;
  while(angle<=-180)
      angle+=360;

	int targetAngle = angle * conv;

	pos_PID anglePID;
	pos_PID_InitController(&anglePID, &angleChange, 0.61, 0.12, 0.07); //Ku = 1.4, Tu = 0.45
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
		currentLeft = nMotorEncoder[driveLFY] - encoderLeft;
		currentRight = nMotorEncoder[driveRFY] - encoderRight;

		angleChange = currentRight - currentLeft;

		//Get output from PID
		angleOutput = pos_PID_StepController(&anglePID);

		//Set motors to angle PID output
		setLeftMotors(-1 * angleOutput);
		setRightMotors(angleOutput);

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
 * Turns clockwise for an angle in degrees
 * @param  angle Angle to turn for (deg)
 */
void turn_SBallsy(long angle){
		//Save left and right quad values instead of setting them to zero
	const long encoderLeft = nMotorEncoder[driveLFY], encoderRight = nMotorEncoder[driveRFY];

	//Total angle change since start
	float angleChange = 0, lastAngle = 0;

	//Conversion between encoder degrees and base_link degrees
	const float conv = 12.88361;

	//Fix angle
	while(angle>180)
      angle-=360;
  while(angle<=-180)
      angle+=360;

	int targetAngle = angle * conv;
		//If angle PID controller is at target
	bool atTarget = false;
long currentLeft, currentRight;
	//Angle that is "close enough" to target
	const int atTargetAngle = 10;
	while(!atTarget){
		//Calculate distance displacement
		currentLeft = nMotorEncoder[driveLFY] - encoderLeft;
		currentRight = nMotorEncoder[driveRFY] - encoderRight;

		angleChange = currentRight - currentLeft;
		setLeftMotors(-1 * sgn(targetAngle) * 127);
		setRightMotors(sgn(targetAngle) * 127);
		atTarget = angleChange > targetAngle - atTargetAngle && angleChange < targetAngle + atTargetAngle;
}
	setLeftMotors(sgn(targetAngle) * 127);
	setRightMotors(-1 * sgn(targetAngle) * 127);
	wait1Msec(20);
	setAllDriveMotors(0);

}

void turn_Ballsy(long angle)
{
	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = nMotorEncoder[driveLFY], encoderRight = nMotorEncoder[driveRFY];

	//Total angle change since start
	float angleChange = 0, lastAngle = 0;

	//Conversion between encoder degrees and base_link degrees
	const float conv = 12.88361;

	//Fix angle
	while(angle>180)
      angle-=360;
  while(angle<=-180)
      angle+=360;

	int targetAngle = angle * conv;

	pos_PID anglePID;
	pos_PID_InitController(&anglePID, &angleChange, 0.61, 0.12, 0.07); //Ku = 1.4, Tu = 0.45
	pos_PID_SetTargetPosition(&anglePID, targetAngle);

	//If angle PID controller is at target
	bool atTarget = false;

	//Angle that is "close enough" to target
	const int atTargetAngle = 10;

	//Threshold for not moving
	const int threshold = 2;

	//Timer for being at target
	timer atTargetTimer, exitTimer;
	timer_Initialize(&atTargetTimer);
	timer_Initialize(&exitTimer);

	//Timeout period (ms)
	const int timeoutPeriod = 250;

	//Current left and right quad displacements
	long currentLeft, currentRight;

	//Distance and angle PID output
	int angleOutput;

	while (!atTarget)
	{
		//Calculate distance displacement
		currentLeft = nMotorEncoder[driveLFY] - encoderLeft;
		currentRight = nMotorEncoder[driveRFY] - encoderRight;

		angleChange = currentRight - currentLeft;

		//Get output from PID
		angleOutput = pos_PID_StepController(&anglePID);

		//Set motors to angle PID output
		setLeftMotors(-1 * angleOutput);
		setRightMotors(angleOutput);

		//Place mark if we're close enough to the target angle
		if (fabs(targetAngle - angleChange) <= atTargetAngle)
		{
			timer_PlaceHardMarker(&atTargetTimer);
			timer_PlaceHardMarker(&exitTimer);
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
		if (timer_GetDTFromHardMarker(&atTargetTimer) >= timeoutPeriod ||
				timer_GetDTFromHardMarker(&exitTimer) >= 50)
		{
			break;
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

	turn_SBallsy(deg - theta);
}

#endif //TURNINGFUNCTIONS_C_INCLUDED
