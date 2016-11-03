#ifndef MOTORCONTROL_C_INCLUDED
#define MOTORCONTROL_C_INCLUDED

bool dumpIntake();
bool driveStraight(const int distance, int swingTheta = 0);
bool turn(const int angle);

//Barely used type to return two values
typedef struct distanceAndAngle_t
{
	float length;
	float theta;
} distanceAndAngle;

void setLeftMotors(const int powerValue)
{
	motor[leftDrive11] = powerValue;
	motor[leftDrive12] = powerValue;
	motor[leftDrive2122] = powerValue;
}

void setRightMotors(const int powerValue)
{
	motor[rightDrive11] = powerValue;
	motor[rightDrive12] = powerValue;
	motor[rightDrive21] = powerValue;
	motor[rightDrive22] = powerValue;
}

void setAllDriveMotors(const int power)
{
	setLeftMotors(power);
	setRightMotors(power);
}

void setIntakeMotors(const int power)
{
	motor[intakeMotors] = power;
}

void setLiftMotors(const int power)
{
	motor[liftMotors] = power;
}

/*
Keeps the intake closed
 */
#warning "keepIntakeClosed"
task keepIntakeClosed()
{
	//Intake PID
	pos_PID pid;

	//PID target position
	int targetPos, prevPos = 0;

	//Target timeout and sensor deadband
	const int timeout = 200, deadband = 10;

	//Timer for reaching target
	timer t;
	timer_Initialize(&t);

	//Start to close intake
	setIntakeMotors(30);

	//Loop until at target
	do
	{
		targetPos = SensorValue[intakePot];

		//We're at target if the intake hasn't closed more in the timeout period
		if (targetPos >= prevPos + deadband && targetPos <= prevPos - deadband)
		{
			timer_PlaceMarker(&t);
		}
	} while (timer_GetDTFromMarker(&t) <= timeout);

	//Use small bias to start
	pos_PID_InitController(&pid, intakePot, 0, 0, 0, 20);
	pos_PID_SetTargetPosition(&pid, targetPos);

	while (true)
	{
		setIntakeMotors(pos_PID_StepController(&pid));
		wait1Msec(15);
	}
}

/*
Keeps the intake open
 */
#warning "keepIntakeOpen"
task keepIntakeOpen()
{
	//Intake PID
	pos_PID pid;

	//PID target position
	int targetPos = 200, prevPos = 0;

	//Target timeout and sensor deadband
	const int timeout = 200, deadband = 5;

	//Timer for reaching target
	timer t;
	timer_Initialize(&t);

	//Start to close intake
	setIntakeMotors(30);

	pos_PID_InitController(&pid, intakePot, 0, 0, 0);
	pos_PID_SetTargetPosition(&pid, targetPos);

	while (true)
	{
		setIntakeMotors(pos_PID_StepController(&pid));
		wait1Msec(15);
	}
}

/*
Dumps the intake over the fence
@return bool Whether the operation was successful
 */
#warning "dumpIntake"
// bool dumpIntake()
// {
// 	//Turn so our back faces the fence
// 	short currentAngle;
// 	BCI_lockSem(std_msgSem, "dumpIntake")
// 	{
// 		currentAngle = std_msg[STD_MSG_EST_THETA];
// 		BCI_unlockSem(std_msgSem, "dumpIntake")
// 	}
//
// 	turn(90 - currentAngle);
//
// 	//Communicate with pi and determine if there is anything blocking our way to
// 	//the fence. If there is, drop what we have and intake whats in our way, and
// 	//dump that. Then, take what we dropped and dump it as well. If there isn't,
// 	//drive back and dump. Lastly, set currentStarTotal to 0.
//
// 	//Loop to clear what's behind us
// 	bool keepGoing = true;
// 	while (keepGoing)
// 	{
// 		sendSPCMsg();
//
// 		//Wait until reply
// 		while (!mpcMsgFlag) { wait1Msec(15); }
//
// 		BCI_lockSem(mpc_msgSem, "dumpIntake")
// 		{
// 			short xDemand, yDemand, pickup;
//
// 			xDemand = mpc_msg[MPC_MSG_X_COORD];
// 			yDemand = mpc_msg[MPC_MSG_Y_COORD];
// 			pickup = mpc_msg[MPC_MSG_PICKUP];
//
// 			BCI_unlockSem(mpc_msgSem, "dumpIntake")
//
// 			switch (pickup)
// 			{
// 				case MPC_MSG_PICKUP_CLEAR:
// 					//Nothing in our way, drive back and dump
// 					keepGoing = false;
// 					break;
//
// 				case MPC_MSG_PICKUP_STAR:
// 					//Star in our way, drop what we have and intake it
// 					//See if there are any other objects in our way
// 					//Wait to score stars until we have a full intake
// 					break;
//
// 				case MPC_MSG_PICKUP_CUBE:
// 					//Cube in our way, drop what we have a score it
// 					//See if there are any other objects in our way
// 					break;
//
// 				default:
// 					break;
// 			}
// 		}
// 	}
//
// 	//Nothing else behind us, score what we put down
//
// 	return true;
// }
bool dumpIntake()
{
	//Turn so our back faces the fence
	short currentAngle;
	BCI_lockSem(std_msgSem, "dumpIntake")
	{
		currentAngle = std_msg[STD_MSG_EST_THETA];
		BCI_unlockSem(std_msgSem, "dumpIntake")
	}

	turn(90 - currentAngle);

	//Pick up stars, drive back and dump
	startTask(keepIntakeClosed);
	setLiftMotors(127);
	setAllDriveMotors(-127);

	bool keepGoing = true;
	while (keepGoing)
	{
		BCI_lockSem(std_msgSem, "dumpIntake")
		{
			//Back up until we're close to the fence
			keepGoing = std_msg[STD_MSG_EST_Y] >= 65;
			BCI_unlockSem(std_msgSem, "dumpIntake");
		}

		wait1Msec(5);
	}

	setAllDriveMotors(0);

	keepGoing = true;
	while (keepGoing)
	{
		//Raise the lift until we're just past vertical
		keepGoing = SensorValue[liftPot] >= 200;
		wait1Msec(5);
	}

	setLiftMotors(0);

	stopTask(keepIntakeClosed);
	startTask(keepIntakeOpen);

	return true;
}

/*
Drives in a straight line for a distance
@param distance Distance to drive for
@param swingTheta Angle between left and right sides (make nonzero for a swing turn)
@return Whether the operation was successful
*/
#warning "driveStraight"
bool driveStraight(const int distance, int swingTheta)
{
	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total distance elapsed since start and total angle change since start
	float distanceElapsed = 0, angleChange = 0;

	//Target distance for the distance PID controller
	//Angle PID controller's target is 0
	const int targetDistance = distance;

	pos_PID distancePID, anglePID;

	pos_PID_InitController(&distancePID, &distanceElapsed, 0.1, 0, 0);
	pos_PID_InitController(&anglePID, &angleChange, 0.1, 0, 0);

	pos_PID_SetTargetPosition(&distancePID, targetDistance);
	pos_PID_SetTargetPosition(&anglePID, swingTheta);

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

		//Angle change doesn't need to be a real angle, just the difference in
		//displacements
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

		wait1Msec(15);
	}

	setAllDriveMotors(0);

	return true;
}

/*
Turns to an angle
@param angle Angle to turn to
@return Whether the operation was successful
*/
#warning "turn"
bool turn(const int angle)
{
	//Save left and right quad values instead of setting them to zero
	long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total angle change since start
	float angleChange = 0;

	//Radius of robot
	const float robotRadius = 10.21875;
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

		//Angle change doesn't need to be a real angle, just the difference in
		//displacements
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

		wait1Msec(15);
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
#warning "computeDistanceToPoint"
float computeDistanceToPoint(const int x, const int y)
{
	BCI_lockSem(std_msgSem, "computeDistanceToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];
		return sqrt((xDiff * xDiff) + (yDiff * yDiff));

		BCI_unlockSem(std_msgSem, "computeDistanceToPoint")
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
#warning "computeAngleToPoint"
float computeAngleToPoint(const int x, const int y)
{
	BCI_lockSem(std_msgSem, "computeAngleToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];

		//Compute difference in angle
		return (atan2(yDiff, xDiff) * (180 / PI)) - std_msg[STD_MSG_EST_THETA];

		BCI_unlockSem(std_msgSem, "computeAngleToPoint")
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
#warning "computeDistanceAndAngleToPoint"
distanceAndAngle* computeDistanceAndAngleToPoint(const int x, const int y)
{
	distanceAndAngle out;

	BCI_lockSem(std_msgSem, "computeDistanceAndAngleToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];
		out.length = sqrt((xDiff * xDiff) + (yDiff * yDiff));

		//Compute difference in angle
		out.theta = (atan2(yDiff, xDiff) * (180 / PI)) - std_msg[STD_MSG_EST_THETA];

		BCI_unlockSem(std_msgSem, "computeDistanceAndAngleToPoint")
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
#warning "moveToPoint"
bool moveToPoint(const int x, const int y, int offset = 0)
{
	distanceAndAngle *temp = computeDistanceAndAngleToPoint(x, y);

	turn(temp->theta);
	driveStraight(temp->length - offset);

	return true;
}

/*
Picks up multiple stars
@param x X coordinates
@param y Y coordinates
@return Whether the operation was successful
 */
#warning "pickUpStars"
bool pickUpStars(const short *x, const short *y)
{
	const int intakeLength = 18;

	//If the stars are along the wall, we need to swing turn in and drive along
	//the wall to get them, then close the intake before we hit the wall
	if (x[0] >= 144 - intakeLength ||
			x[0] <= intakeLength ||
			y[0] <= intakeLength ||
			y[0] >= 72 - intakeLength)
	{
		//Drive to a point in front of one end of the wall
		//For the left and right walls, drive next to the fence
		//For the south wall, pick the closest corner
		//
		//Turn to face the wall
		//
		//Swing turn into being parallel to the wall
		//
		//Drive along the length of the wall to pool objects
		//
		//Close the intake and score
	}
	//Otherwise, we should drive around with our intake open to get them
	else
	{
		//Drive around hitting each star
		//
		//Close the intake and score
	}

	//Dump stars
	dumpIntake();

	//Tell pi we are done
	sendMPCMsg();

	return true;
}

/*
Picks up a cube and scores it
@param x X coordinate of cube
@param y Y coordinate of cube
@return Whether the operation was successful
 */
#warning "pickUpCube"
bool pickUpCube(const int x, const int y)
{
	//Move to cube
	moveToPoint(x, y);

	//Close intake
	startTask(keepIntakeCLosed);

	//Dump cube
	dumpIntake();

	//Tell pi we are done
	sendMPCMsg();

	return true;
}

#endif //MOTORCONTROL_C_INCLUDED
