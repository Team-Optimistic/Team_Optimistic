#ifndef MOTORCONTROL_C_INCLUDED
#define MOTORCONTROL_C_INCLUDED

bool dumpIntake();
bool driveStraight(const int distance);
bool turn(const int angle);

//Barely used type to return two values
typedef struct distanceAndAngle_t
{
	float length;
	float theta;
} distanceAndAngle;

void initSensors()
{
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;
	SensorValue[liftIME] = 0;
}

void setLeftMotors(const int powerValue)
{
	motor[driveLFY] = powerValue;
	motor[driveLB] = powerValue;
}

void setRightMotors(const int powerValue)
{
	motor[driveRFY] = powerValue;
	motor[driveRB] = powerValue;
}

void setAllDriveMotors(const int power)
{
	setLeftMotors(power);
	setRightMotors(power);
}

void setIntakeMotors(const int power)
{
	motor[intakeY] = power;
}

void setLiftMotors(const int power)
{
	motor[liftLO] = power;
	motor[liftLI] = power;
	motor[liftRO] = power;
	motor[liftRI] = power;
}

/*
Handles the intake and lift
 */
enum intakeState
{
	INTAKE_OPEN,
	INTAKE_CLOSED,
	INTAKE_REST
};

enum liftState
{
	LIFT_UP,
	LIFT_DOWN,
	LIFT_REST
};

intakeState intakeAndLiftTask_intakeState = INTAKE_REST;
intakeState intakeAndLiftTask_liftState = LIFT_REST;

task intakeAndLiftTask()
{
	pos_PID intakePID, liftPID;

	pos_PID_InitController(&intakePID, intakePot, 0.3, 0.2, 0.1, 0);
	pos_PID_InitController(&liftPID, liftRI, 0.3, 0.2, 0.1, -10);

	pos_PID_SetTargetPosition(&intakePID, 2000); //Open position

	while (true)
	{
		switch (intakeAndLiftTask_intakeState)
		{
			case INTAKE_OPEN:
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_CLOSED:
				setIntakeMotors(-20);
				break;

			case INTAKE_REST:
				setIntakeMotors(0);
				break;
		}

		if (SensorValue[liftStopButton])
			nMotorEncoder[liftRI] = 0;

		switch (intakeAndLiftTask_liftState)
		{
			case LIFT_UP:
				pos_PID_SetTargetPosition(&liftPID, 200); //Up position
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_DOWN:
				pos_PID_SetTargetPosition(&liftPID, 0);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_REST:
				setLiftMotors(0);
				break;
		}

		wait1Msec(15);
	}
}

/*
Dumps the intake over the fence
@return bool Whether the operation was successful
 */
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
	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
	intakeAndLiftTask_liftState = LIFT_UP;
	startTask(intakeAndLiftTask);

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

	//Wait until lift is up
	keepGoing = true;
	while (keepGoing)
	{
		keepGoing = nMotorEncoder[liftRI] >= 200;
		wait1Msec(5);
	}

	intakeAndLiftTask_intakeState = INTAKE_CLOSED;

	return true;
}

/*
Drives in a straight line for a distance
@param distance Distance to drive for
@param swingTheta Angle between left and right sides (make nonzero for a swing turn)
@return Whether the operation was successful
*/
bool driveStraight(const long distance)
{
	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total distance elapsed since start and total angle change since start
	float distanceElapsed = 0, angleChange = 0;
	float lastDistance = 0;

	//Target distance for the distance PID controller
	//Angle PID controller's target is 0
	const int targetDistance = distance;

	pos_PID distancePID, anglePID;

	if (distance <= 800)
	{
		pos_PID_InitController(&distancePID, &distanceElapsed, 0.3, 0.2, 0.1);
		pos_PID_InitController(&anglePID, &angleChange, 0.5, 0.25, 0);
	}
	else
	{
		pos_PID_InitController(&distancePID, &distanceElapsed, 0.3, 0.2, 0.2);
		pos_PID_InitController(&anglePID, &angleChange, 0.5, 0.5, 0);
	}

	pos_PID_SetTargetPosition(&distancePID, targetDistance);
	pos_PID_SetTargetPosition(&anglePID, 0);

	//If distance PID controller is at target
	bool atTarget = false;

	//Distance that is "close enough" to target
	const int atTargetDistance = 15;

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
		//Place mark if we haven't moved much
		else if (fabs(distanceElapsed - lastDistance) <= threshold)
		{
			timer_PlaceHardMarker(&atTargetTimer);
		}
		else
		{
			timer_ClearHardMarker(&atTargetTimer);
		}

		lastDistance = distanceElapsed;

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
bool turn(const long angle)
{
	//Save left and right quad values instead of setting them to zero
	long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total angle change since start
	float angleChange = 0, lastAngle = 0;

	//Radius of robot
	const float thetaConv = 12.75993;

	//Target angle
	int targetAngle = angle * thetaConv;

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

	return true;
}

/*
Computes the distance to a point
@param x X coordinate of other point
@param y Y coordinate of other point
@return distance to point
*/
float computeDistanceToPoint(const long x, const long y)
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
float computeAngleToPoint(const long x, const long y)
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
@param x   X coordinate of other point
@param y   Y coordinate of other point
@param out Struct to write result to
*/
void computeDistanceAndAngleToPoint(const long x, const long y, distanceAndAngle *out)
{
	//If no lock, return empty type
	out->length = 0;
	out->theta = 0;

	BCI_lockSem(std_msgSem, "computeDistanceAndAngleToPoint")
	{
		//Compute difference in distance
		writeDebugStreamLine("comp: x: %d, estx: %d, y: %d, esty: %d, estth: %d", x, std_msg[STD_MSG_EST_X], y, std_msg[STD_MSG_EST_Y], std_msg[STD_MSG_EST_THETA]);
		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];
		out->length = sqrt((xDiff * xDiff) + (yDiff * yDiff));

		//Compute difference in angle
		out->theta = (atan2(yDiff, xDiff) * (180 / PI)) - std_msg[STD_MSG_EST_THETA];

		BCI_unlockSem(std_msgSem, "computeDistanceAndAngleToPoint")
	}
}

/*
Turns and drives to a point
@param x X coordinate to move to
@param y Y coordinate to move to
@param offset Backward offset from final distance to point
@return Whether the operation was successful
*/
bool moveToPoint(const long x, const long y, long offset = 0)
{
	distanceAndAngle temp;
	computeDistanceAndAngleToPoint(x, y, &temp);

	writeDebugStreamLine("turning %1.2f", temp.theta);
	turn(temp.theta);

	writeDebugStreamLine("driving %1.2f", temp.length - offset);
	driveStraight(temp.length - offset);
	writeDebugStreamLine("done");

	return true;
}

/*
Picks up multiple stars
@param x X coordinates
@param y Y coordinates
@return Whether the operation was successful
 */
bool pickUpStars(const long *x, const long *y)
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

	return true;
}

/*
Picks up a cube and scores it
@param x X coordinate of cube
@param y Y coordinate of cube
@return Whether the operation was successful
 */
bool pickUpCube(const long x, const long y)
{
	//Move to cube
	moveToPoint(x, y);

	//Close intake
	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
	startTask(intakeAndLiftTask);

	//Dump cube
	dumpIntake();

	return true;
}

#endif //MOTORCONTROL_C_INCLUDED
