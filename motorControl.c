#ifndef MOTORCONTROL_C_INCLUDED
#define MOTORCONTROL_C_INCLUDED

void initSensors()
{
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;
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
 * @param offset    Backward offset from final distance to point
 * @param backwards Whether to move to the point backwards
 */
void moveToPoint(const long x, const long y, long offset = 0, bool backwards = false)
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
	if ((limit = doesTurnCollide(temp.theta)) != 0)
	{
		//Turn as far as we can without hitting anything
		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("turning %1.2f", limit);
		#endif
		turn(limit);

		//Figure out how far we have to drive until we can turn the rest of the way
		long mmToDrive = 0;

		statePack sp;
		BCI_lockSem(std_msgSem, "moveToPoint")
		{
			sp.x = std_msg[STD_MSG_EST_X];
			sp.y = std_msg[STD_MSG_EST_Y];
			sp.theta = std_msg[STD_MSG_EST_THETA];
			BCI_unlockSem(std_msgSem, "moveToPoint")
		}

		while (true)
		{
			mmToDrive += 10; //Add 10 mm to drive distance
			sp_Translate(&sp, 10, sp.theta); //Move 10 mm into the future

			//Check if we still collide
			if (!doesTurnCollideSP(&sp, computeAngleToPoint(x, y)))
			{
				//Break if we don't collide
				break;
			}
		}

		//Drive the calculated distance
		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("driving %1.2f", mmToDrive);
		#endif
		driveStraight(mmToDrive);

		//Turn the rest of the angle
		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("turning %1.2f", computeAngleToPoint(x, y));
		#endif
		turn(computeAngleToPoint(x, y));

		//Drive to the point
		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("driving %1.2f", computeDistanceToPoint(x, y));
		#endif
		driveStraight(computeDistanceToPoint(x, y));
	}
	//Else, we don't hit anything
	else
	{
		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("turning %1.2f", temp.theta);
		#endif
		turn(temp.theta);

		#ifdef MOVETOPOINT_DEBUG
			writeDebugStreamLine("driving %1.2f", temp.length - offset);
		#endif
		driveStraight(temp.length - offset);
	}

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("done");
	#endif
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
			moveToPoint(3100, 1118); //Center of right segment
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
	vel_PID intakePID, drivePID;

	//Save left and right quad values instead of setting them to zero
	const long encoderLeft = SensorValue[leftQuad], encoderRight = SensorValue[rightQuad];

	//Total distance elapsed since start and total angle change since start
	float distanceElapsed = 0, angleChange = 0;
	float lastDistance = 0;

  //Conversion between encoder degrees and base_link mm
  const float conv = 1.311250;

	//Target distance for the distance PID controller
	//Angle PID controller's target is 0
	int targetDistance = -610 * conv;

	pos_PID distancePID, anglePID;

	pos_PID_InitController(&distancePID, &distanceElapsed, 0.2, 0.2, 0.1);
	pos_PID_InitController(&anglePID, &angleChange, 0.5, 0.25, 0);

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
		angleChange = currentLeft - currentRight;

		//Get output from both PID's
		distOutput = pos_PID_StepController(&distancePID);
		angleOutput = pos_PID_StepController(&anglePID);

		//Set motors to distance PID output with correction from angle PID
		setLeftMotors(distOutput + angleOutput);
		setRightMotors(distOutput - angleOutput);

		writeDebugStreamLine("%d",pos_PID_GetError(&distancePID));

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
	writeDebugStreamLine("target: %1.2f", targetDistance);
}

#endif //MOTORCONTROL_C_INCLUDED
