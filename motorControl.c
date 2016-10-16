#ifndef MOTORCONTROL_C_INCLUDED
#define MOTORCONTROL_C_INCLUDED

bool intakeStar();
bool intakeCube();
bool dumpIntake();
bool dumpStars();

//Number of stars in the intake
static int currentStarTotal = 0;

//Barely used type to return two values
typedef struct distanceAndAngle_t
{
	float length;
	float theta;
} distanceAndAngle;

void setLeftMotors(const int powerValue)
{
	motor[leftMotor] = powerValue;
}

void setRightMotors(const int powerValue)
{
	motor[rightMotor] = powerValue;
}

void setAllDriveMotors(const int power)
{
	setLeftMotors(power);
	setRightMotors(power);
}

void setIntakeMotors(const int power)
{
}

/*
Intakes a star
@return Whether the operation was successful
*/
#warning "IntakeStar"
bool intakeStar()
{
	if (currentStarTotal == 0)
	{
	}
	else
	{
	}

	currentStarTotal++;

	return true;
}

/*
Intakes a cube
@return Whether the operation was successful
 */
#warning "IntakeCube"
bool intakeCube()
{
	if (currentStarTotal != 0)
	{
		dumpStars();
	}
	else
	{
	}
}

/*
Dumps the intake over the fence
@return bool Whether the operation was successful
 */
#warning "DumpIntake"
bool dumpIntake()
{
	//Communicate with pi and determine if there is anything blocking our way to
	//the fence. If there is, drop what we have and intake whats in our way, and
	//dump that. Then, take what we dropped and dump it as well. If there isn't,
	//drive back and dump. Lastly, set currentStarTotal to 0.

	bool keepGoing = true;
	short msgIDCounter = 0;

	while (keepGoing)
	{
		sendGetBehindRequest(msgIDCounter);
		msgIDCounter++;

		BCI_lockSem(spc_msgSem, "dumpIntake")
		{
			short xDemand, yDemand, pickup;

			xDemand = spc_msg[SPC_MSG_X_COORD];
			yDemand = spc_msg[SPC_MSG_Y_COORD];
			pickup = spc_msg[SPC_MSG_PICKUP];

			BCI_unlockSem(spc_msgSem, "dumpIntake")

			switch (pickup)
			{
				case SPC_MSG_PICKUP_CLEAR:
					//Nothing in our way, drive back and dump
					break;

				case SPC_MSG_PICKUP_STAR:
					//Star in our way, drop what we have and intake it
					//See if there are any other objects in our way
					//Wait to score stars until we have a full intake

				case SPC_MSG_PICKUP_CUBE:
					//Cube in our way, drop what we have a score it
					//See if there are any other objects in our way
			}
		}
	}

	return true;
}

/*
Randomly selects a strategy based on a probability distribution and dumps based
on the strategy
@return bool Whether the operation was successful
 */
#warning "DumpStars"
bool dumpStars()
{
	//Randomly select a strategy
	//Depending on the strategy and currentStarTotal, either
	// - Dump stars with dumpIntake, or
	// - Keep stars

	return true;
}

/*
Drives in a straight line for a distance
@param distance Distance to drive for
@return Whether the operation was successful
*/
#warning "DriveStraight"
bool driveStraight(const int distance)
{
	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total distance elapsed since start and total angle change since start
	float distanceElapsed = 0, angleChange = 0;

	//Target distance for the distance PID controller
	//Angle PID controller's target is 0
	const int targetDistance = distance;

	pos_PID distancePID, anglePID;

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
			timer_PlaceHardMarker(&atTargetTimer);
		}
		else
		{
			timer_ClearHardMarker(&atTargetTimer);
		}

		//If we've been close enough for long enough, we're there
		if (timer_GetDTFromHardMarker(&atTargetTimer) >= timeoutPeriod)
		{
			atTarget = true;
		}
	}

	setAllDriveMotors(0);

	return true;
}

/*
Turns to an angle
@param angle Angle to turn to
@return Whether the operation was successful
*/
#warning "Turn"
bool turn(const int angle)
{
	//Save left and right quad values instead of setting them to zero
	long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total angle change since start
	float angleChange = 0;

	//Radius of robot
	const float robotRadius = 11.5;
	const float angleScale = 0.017453 * robotRadius; //2pi*radius*(theta/360)=encoder

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
		angleChange = currentRight / angleScale;

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
		else
		{
			timer_ClearHardMarker(&atTargetTimer);
		}

		//If we've been close enough for long enough, we're there
		if (timer_GetDTFromHardMarker(&atTargetTimer) >= timeoutPeriod)
		{
			atTarget = true;
		}
	}

	setAllDriveMotors(0);

	return true;
}

/*
Computes the distance to a point
@param x X coordinate of other point
@param y Y coordinate of other point
@return distance to point
*/
#warning "ComputeDistanceToPoint"
float computeDistanceToPoint(const int x, const int y)
{
	BCI_lockSem(msgSem, "computeDistanceToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - msg[MSG_EST_X], yDiff = y - msg[MSG_EST_Y];
		return sqrt((xDiff * xDiff) + (yDiff * yDiff));

		BCI_unlockSem(msgSem, "computeDistanceToPoint")
	}

	//If no lock, return no distance
	return 0;
}

/*
Computes the angle to a point
@param x X coordinate of other point
@param y Y coordinate of other point
@return angle to point
*/
#warning "ComputeAngleToPoint"
float computeAngleToPoint(const int x, const int y)
{
	BCI_lockSem(msgSem, "computeAngleToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - msg[MSG_EST_X], yDiff = y - msg[MSG_EST_Y];

		//Compute difference in angle
		return (atan2(yDiff, xDiff) * (180 / PI)) - msg[MSG_EST_THETA];

		BCI_unlockSem(msgSem, "computeAngleToPoint")
	}

	//If no lock, return no angle
	return 0;
}

/*
Computes the distance and angle from current location to a point
@param x X coordinate of other point
@param y Y coordinate of other point
@return distance and angle to point
*/
#warning "ComputeDistanceAndAngleToPoint"
distanceAndAngle* computeDistanceAndAngleToPoint(const int x, const int y)
{
	distanceAndAngle out;

	BCI_lockSem(msgSem, "computeDistanceAndAngleToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - msg[MSG_EST_X], yDiff = y - msg[MSG_EST_Y];
		out.length = sqrt((xDiff * xDiff) + (yDiff * yDiff));

		//Compute difference in angle
		out.theta = (atan2(yDiff, xDiff) * (180 / PI)) - msg[MSG_EST_THETA];

		BCI_unlockSem(msgSem, "computeDistanceAndAngleToPoint")
	}

	//If no lock, return empty type
	out.length = 0;
	out.theta = 0;

	return out;
}

/*
Turns and drives to a point
@param x X coordinate to move to
@param y Y coordinate to move to
@param offset Backward offset from final distance to point
@return Whether the operation was successful
*/
#warning "MoveToPoint"
bool moveToPoint(const int x, const int y, int offset = 0)
{
	distanceAndAngle *temp = computeDistanceAndAngleToPoint(x, y);

	turn(temp->theta);
	driveStraight(temp->length - offset);

	return true;
}

/*
Picks up a star
@param x X coordinate of star
@param y Y coordinate of star
@return Whether operation was successful
 */
#warning "PickUpStar"
bool pickUpStar(const int x, const int y)
{
	//Move to slightly behind star
	moveToPoint(x, y, 10);
	intakeStar();

	//Dump stars
	dumpStars();

	return true;
}

/*
Picks up a cube and scores it
@param x X coordinate of cube
@param y Y coordinate of cube
@return Whether the operation was successful
 */
#warning "PickUpCube"
bool pickUpCube(const int x, const int y)
{
	//Dump intake if we have anything, we cannot pick up a cube with stars in the
	//intake
	if (currentStarTotal != 0)
	{
		dumpIntake();
	}

	//Move to slightly behind cube
	moveToPoint(x, y, 10);
	intakeCube();

	//Dump cube
	dumpIntake();

	return true;
}

#endif //MOTORCONTROL_C_INCLUDED
