#ifndef MOTORCONTROL_H_INCLUDED
#define MOTORCONTROL_H_INCLUDED

#include "uartHandler.c"

void setLeftMotors(const int powerValue)
{
	motor[leftMotor] = powerValue;
}

void setRightMotors(const int powerValue)
{
	motor[rightMotor] = powerValue;
}

void setAllMotors(const int power)
{
	setLeftMotors(power);
	setRightMotors(power);
}

bool driveStraight(const int distance)
{
	//Save left and right quad values instead of setting them to zero
	long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total distance elapsed since start and total angle change since start
	float distanceElapsed = 0, angleChange = 0;

	//Target distance for the distance PID controller
	//Angle PID controller's target is 0
	int targetDistance = distance;

	pos_PID distancePID , anglePID;

	pos_PID_InitController(&distancePID, &distanceElapsed, 0, 0, 0);
	pos_PID_InitController(&anglePID, &angleChange, 0, 0, 0);

	pos_PID_SetTargetPosition(&distancePID, targetDistance);
	pos_PID_SetTargetPosition(&anglePID, 0);

	//If distance PID controller is at target
	bool atTarget = false;

	//Distance that is "close enough" to target
	const int atTargetDistance = 5;

	//Timer for being at target
	timer atTargetTimer;
	timer_Initialize(&atTargetTimer);

	//Timeout period (ms)
	const int timeoutPeriod = 250;

	//Current left and right quad displacements
	long currentLeft, currentRight;

	//Distance and angle PID output
	int distOutput, angleOutput;

	while (!atTarget)
	{
		//Calculate distance displacement
		currentLeft = SensorValue[leftQuad] - encoderLeft;
		currentRight = SensorValue[rightQuad] - encoderRight;

		//Overall displacement is the average of left and right displacements
		distanceElapsed = (currentLeft + currentRight) / 2.0;

		//Angle change doesn't need to be a real angle, just the difference in displacements
		angleChange = currentRight - currentLeft;

		//Get output from both PID's
		distOutput = pos_PID_StepController(&distancePID);
		angleOutput = pos_PID_StepController(&anglePID);

		//Set motors to distance PID output with correction from angle PID
		setLeftMotors(distOutput + angleOutput);
		setRightMotors(distOutput - angleOutput);

		//Place mark if we're close enough to the target distance
		if (fabs(targetDistance - distanceElapsed) <= atTargetDistance)
		{
			timer_PlaceMarker(&atTargetTimer);
		}

		//If we've been close enough for long enough, we're there
		if (timer_GetDTFromMarker(&atTargetTimer) > timeoutPeriod)
		{
			atTarget = true;
		}
	}

	setAllMotors(0);

	return true;
}

bool turn(const int angle)
{
	//Save left and right quad values instead of setting them to zero
	long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total angle change since start
	float angleChange = 0;

	//Target angle
	int targetAngle = angle;

	pos_PID anglePID;

	pos_PID_InitController(&anglePID, &angleChange, 0, 0, 0);

	pos_PID_SetTargetPosition(&anglePID, 0);

	//If angle PID controller is at target
	bool atTarget = false;

	//Angle that is "close enough" to target
	const int atTargetAngle = 10;

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

		//Angle change doesn't need to be a real angle, just the difference in displacements
		angleChange = currentRight - currentLeft;

		//Get output from PID
		angleOutput = pos_PID_StepController(&anglePID);

		//Set motors to angle PID output
		setLeftMotors(angleOutput);
		setRightMotors(-1 * angleOutput);

		//Place mark if we're close enough to the target angle
		if (fabs(targetAngle - angleChange) <= atTargetAngle)
		{
			timer_PlaceMarker(&atTargetTimer);
		}

		//If we've been close enough for long enough, we're there
		if (timer_GetDTFromMarker(&atTargetTimer) > timeoutPeriod)
		{
			atTarget = true;
		}
	}

	setAllMotors(0);

	return true;
}

bool moveToPoint(const int x, const int y)
{
	semaphoreLock(msgSem);

	if (bDoesTaskOwnSemaphore(msgSem))
	{
		//Compute difference in distance and drive
		float xDiff = x - msg[1], yDiff = y - msg[2];
		float distance = sqrt((xDiff * xDiff) + (yDiff * yDiff));

		//Compute difference in angle and turn
		float thetaDiff = (atan2(yDiff, xDiff) * (180 / PI)) - msg[3];

		if (bDoesTaskOwnSemaphore(msgSem))
		{
			semaphoreUnlock(msgSem);
		}

		turn(thetaDiff);
		driveStraight(distance);
	}

	return true;
}

#endif //MOTORCONTROL_H_INCLUDED
