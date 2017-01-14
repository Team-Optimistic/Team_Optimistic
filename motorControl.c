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
			BCI_unlockSem(std_msgSem, "dumpIntake");
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
		}
		BCI_unlockSem(std_msgSem, "moveToPoint");

		while (true)
		{
			mmToDrive += 10; //Add 10 mm to drive distance
			sp_Translate(&sp, 10, sp.theta); //Move 10 mm into the future

			//Check if we still collide
			if (!doesTurnCollide(&sp, computeAngleToPoint(x, y)))
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
			moveToPoint(3543, 1118); //Center of left segment
			turnToAbsAngle(180);
			//knock the stars off
			break;

		case FENCE_MIDDLE:
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

#endif //MOTORCONTROL_C_INCLUDED
