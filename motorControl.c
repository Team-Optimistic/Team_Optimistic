#ifndef MOTORCONTROL_C_INCLUDED
#define MOTORCONTROL_C_INCLUDED

#define FENCE_LEFT_X  300
#define FENCE_RIGHT_X 3100
#define FENCE_RIGHT_Y 1118

void initSensors()
{
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;
}

/**
 * Basic movement of scoring
 */
void dumpIntakeBasic()
{
	turnToAbsAngle(180);
	intakeAndLiftTask_liftState = LIFT_UP;
	waitForLift(LIFT_DUMP);
	setAllDriveMotors(-127);
	wait1Msec(200);
	intakeAndLiftTask_intakeState = INTAKE_OPEN;
	wait1Msec(100);
	setAllDriveMotors(0);
	intakeAndLiftTask_liftState = LIFT_DOWN;
}

/**
 * Scores whatever is in the intake
 */
void dumpIntake()
{
	//Close intake and lift up while we move to the fence
	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
	intakeAndLiftTask_liftState = LIFT_UP;
	startTask(intakeAndLiftTask);

	//Turn so our back faces the fence
	turnToAbsAngle(180);

	//Run into the fence
	setAllDriveMotors(-127);

	bool keepGoing = true;
	while (keepGoing)
	{
		BCI_lockSem(std_msgSem, "dumpIntake")
		{
			//Back up until we're close to the fence
			keepGoing = std_msg[STD_MSG_EST_X] >= 1820; //Slightly less than the field height (1828)

			//Open the intake when the lift is partway up
			if (nMotorEncoder[liftRI] >= 200)
			{
				intakeAndLiftTask_liftState = INTAKE_OPEN;
			}
			BCI_unlockSem(std_msgSem, "dumpIntake")
		}

		wait1Msec(5);
	}

	setAllDriveMotors(0);
}

/**
 * Turns and drives to a point
 * @param x         X coordinate of point
 * @param y         Y coordinate of point
 * @param backwards Whether to move to the point backwards
 * @param offset    Backward offset from final distance to point
 */
void moveToPoint(const long x, const long y, bool backwards = false, long offset = 0)
{
	distanceAndAngle temp;
	computeDistanceAndAngleToPoint(x, y, &temp);

	if (backwards)
	{
		temp.theta += 180;
		temp.length *= -1;
	}

	long limit;
	//If we hit something during the turn
	//if ((limit = doesTurnCollide(temp.theta)) != 0)
	//{
	//	//Turn as far as we can without hitting anything
	//	#ifdef MOVETOPOINT_DEBUG
	//		writeDebugStreamLine("movetopoint: turning to limit: %1.2f", limit);
	//	#endif
	//	turn(limit);

	//	//Figure out how far we have to drive until we can turn the rest of the way
	//	long mmToDrive = 0;

	//	statePack sp;
	//	BCI_lockSem(std_msgSem, "moveToPoint")
	//	{
	//		sp.x = std_msg[STD_MSG_EST_X];
	//		sp.y = std_msg[STD_MSG_EST_Y];
	//		sp.theta = std_msg[STD_MSG_EST_THETA];
	//		BCI_unlockSem(std_msgSem, "moveToPoint")
	//	}

	//	while (true)
	//	{
	//		mmToDrive += 10; //Add 10 mm to drive distance
	//		sp_Translate(&sp, 10, sp.theta); //Move 10 mm into the future

	//		//Check if we still collide
	//		if (!doesTurnCollideSP(&sp, computeAngleToPoint(x, y)))
	//		{
	//			//Break if we don't collide
	//			break;
	//		}
	//	}

	//	//Drive the calculated distance
	//	#ifdef MOVETOPOINT_DEBUG
	//		writeDebugStreamLine("movetopoint: driving to limit: %1.2f", mmToDrive);
	//	#endif
	//	driveStraight(mmToDrive);

	//	//Turn the rest of the angle
	//	#ifdef MOVETOPOINT_DEBUG
	//		writeDebugStreamLine("movetopoint: turning to point: %1.2f", computeAngleToPoint(x, y));
	//	#endif
	//	turn(computeAngleToPoint(x, y));

	//	//Drive to the point
	//	#ifdef MOVETOPOINT_DEBUG
	//		writeDebugStreamLine("movetopoint: driving to point: %1.2f", computeDistanceToPoint(x, y));
	//	#endif
	//	driveStraight(computeDistanceToPoint(x, y));
	//}
	////Else, we don't hit anything
	//else
	{
		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("movetopoint: turning all the way: %1.2f", temp.theta);
		#endif
		turn(temp.theta);

		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("movetopoint: driving all the way: %1.2f", temp.length - offset);
		#endif
		driveStraight(temp.length - offset);
	}

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint: done");
	#endif
}

void moveToPoint_Translate(const int x, const int y, bool backwards = false)
{
	long currentX = 0, currentY = 0;

	BCI_lockSem(std_msgSem, "moveToPoint_Translate")
	{
		currentX = std_msg[STD_MSG_EST_X];
		currentY = std_msg[STD_MSG_EST_Y];
		BCI_unlockSem(std_msgSem, "moveToPoint_Translate")
	}
	writeDebugStreamLine("moving x: %d, y: %d", currentX + x, currentY + y);
	moveToPoint(currentX + x, currentY + y, backwards, 0);
}

enum fenceTypes
{
	FENCE_LEFT,
	FENCE_MIDDLE,
	FENCE_RIGHT
};

/**
 * Scores stars off of a fence section
 * @param fence Section of fence to score
 */
void scoreFence(const fenceTypes fence)
{
	distanceAndAngle temp;

	//Load in point in the center of the selected field section
	//Each fence is 1181 mm wide
	switch (fence)
	{
		case FENCE_LEFT:
			break;

		case FENCE_MIDDLE:
			break;

	  case FENCE_RIGHT:
			moveToPoint(FENCE_RIGHT_X, FENCE_RIGHT_Y); //Center of right segment
			turnToAbsAngle(180);
			//knock the stars off
			break;

		default:
			break;
	}
}

/**
 * Picks up multiple stars
 * @param x X coordinates
 * @param y Y coordinates
 */
void pickUpStars(const long *x, const long *y)
{
	const int intakeLength = 458;

	//If the stars are along the wall, we need to swing turn in and drive along
	//the wall to get them, then close the intake before we hit the wall
	if (x[0] >= 3657 - intakeLength ||
			x[0] <= intakeLength ||
			y[0] <= intakeLength ||
			y[0] >= 1828 - intakeLength)
	{
		//Drive to a point in front of one end of the wall
		//For the left and right walls, drive next to the fence
		//For the south wall, pick the closest corner
		//Turn to face the wall
		//Swing turn into being parallel to the wall
		//Drive along the length of the wall to pool objects
		//Close intake
		//
		//Or, maybe we should approach the wall flat
		//Close the intake and back up at set velocities
		//Split up stars wider than 50" apart into multiple dumps
	}
	//Otherwise, we should drive around with our intake open to get them
	else
	{
		//Drive around hitting each star
		//Close intake
	}

	//Dump stars
	dumpIntake();
}

/**
 * Picks up a cube and scores it
 * @param x X coordinate of cube
 * @param y Y coordinate of cube
 */
void pickUpCube(const long x, const long y)
{
	//Move to cube
	moveToPoint(x, y, 100);

	//Close intake
	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
	intakeAndLiftTask_liftState = LIFT_WAIT;
	startTask(intakeAndLiftTask);

	setLiftMotors(-100);
	wait1Msec(500);
	intakeAndLiftTask_liftState = LIFT_DOWN;

	bool keepGoing = false;
	while (!keepGoing)
	{
		keepGoing = SensorValue[intakePot] <= 850;
		wait1Msec(5);
	}

	//Dump cube
	dumpIntake();

	intakeAndLiftTask_intakeState = INTAKE_OPEN;
	intakeAndLiftTask_liftState = LIFT_DOWN;
}

/**
 * Pick up stars against the wall during skills
 */
void cheeseThoseStars()
{
	pos_PID intakePPID, drivePPID;
	vel_PID intakeVPID, driveVPID;

	float distanceElapsed = 0, lastDistance = 0;
	const int targetDistance = -610;

	float intakeOutput = 0, driveOutput = 0;

	//We need control of the intake
	intakeAndLiftTask_intakeState = INTAKE_WAIT;

	pos_PID_InitController(&intakePPID, intakePot, 0.2, 0.1, 0, 0);
	pos_PID_SetTargetPosition(&intakePPID, INTAKE_CLOSED_VAL);
	vel_PID_InitController(&intakeVPID, &intakeOutput, 0.2, 0.1);
	vel_PID_SetTargetVelocity(&intakeVPID, -10);

	pos_PID_InitController(&drivePPID, &distanceElapsed, 0.2, 0.2, 0.1);
	pos_PID_SetTargetPosition(&drivePPID, targetDistance);
	vel_PID_InitController(&driveVPID, &driveOutput, 0.2, 0.1);
	vel_PID_SetTargetVelocity(&driveVPID, -10);

	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

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

	//Final control signal
	int intakeSignal, driveSignal;

	while (!atTarget)
	{
		//Calculate distance displacement
		currentLeft = SensorValue[leftQuad] - encoderLeft;
		currentRight = SensorValue[rightQuad] - encoderRight;

		//Overall displacement is the average of left and right displacements
		distanceElapsed = (currentLeft + currentRight) / 2.0;

		//Get output
		intakeOutput = pos_PID_StepController(&intakePPID);
		intakeSignal = vel_PID_StepController(&intakeVPID);
		driveOutput = pos_PID_StepController(&drivePPID);
		driveSignal = vel_PID_StepController(&driveVPID);

		//Set motors
		setIntakeMotors(intakeSignal);
		setLeftMotors(driveSignal);
		setRightMotors(driveSignal);

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
}

#endif //MOTORCONTROL_C_INCLUDED
