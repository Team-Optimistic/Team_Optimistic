#ifndef MOTORCONTROL_C_INCLUDED
#define MOTORCONTROL_C_INCLUDED

#define FENCE_LEFT_X   610
#define FENCE_MIDDLE_X 914
#define FENCE_RIGHT_X  1524
#define FENCE_ANY_Y    610

void initSensors()
{
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;
}

/**
 * Basic movement of scoring
 */
void dumpIntake(bool shouldNotTurn = false, bool shouldNotPutLiftDown = false)
{
	if (!shouldNotTurn)
		turnToAbsAngle(-90);
	intakeAndLiftTask_liftState = LIFT_UP;
	waitForLift(LIFT_DUMP);
	setAllDriveMotors(-127);
	wait1Msec(200);
	intakeAndLiftTask_intakeState = INTAKE_OPEN;
	wait1Msec(100);
	setAllDriveMotors(0);
	wait1Msec(100);
	waitForLift(LIFT_UP);
	wait1Msec(1000);
	if (!shouldNotPutLiftDown)
		intakeAndLiftTask_liftState = LIFT_DOWN;
	driveStraight_Ballsy(300);
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

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint: turning all the way: %1.2f", temp.theta);
	#endif

	turn(temp.theta);
	wait1Msec(1000);

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint: driving all the way: %1.2f", temp.length - offset);
	#endif

	driveStraight(temp.length - offset);

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint: done");
	#endif
}

/**
 * Turns and drives to a point recklessly
 * @param x         X coordinate of point
 * @param y         Y coordinate of point
 * @param backwards Whether to move to the point backwards
 * @param offset    Backward offset from final distance to point
 */
void moveToPoint_Ballsy(const long x, const long y, bool backwards = false, long offset = 0)
{
	distanceAndAngle temp;
	computeDistanceAndAngleToPoint(x, y, &temp);

	if (backwards)
	{
		temp.theta += 180;
		temp.length *= -1;
	}

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint_ballsy: turning all the way: %1.2f", temp.theta);
	#endif

	turn_Ballsy(temp.theta);

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint_ballsy: driving all the way: %1.2f", temp.length - offset);
	#endif

	driveStraight_Ballsy(temp.length - offset);

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("movetopoint_ballsy: done");
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

	#ifdef MOVETOPOINT_DEBUG
		writeDebugStreamLine("moving x: %d, y: %d", currentX + x, currentY + y);
	#endif

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
	//distanceAndAngle temp;

	//Load in point in the center of the selected field section
	//Each fence is 1181 mm wide
	switch (fence)
	{
		case FENCE_LEFT:
			break;

		case FENCE_MIDDLE:
			break;

	  case FENCE_RIGHT:
			break;

		default:
			break;
	}
}

/**
 * Picks up a star
 * @param x X coordinates
 * @param y Y coordinates
 */
 // true if against wall
bool pickUp(const long x, const long y , bool isCube)
{

	int half = 1790;//half field in mm
	int quarter = 895;//quarter field
	int fakeX = x - half;
	int fakeY = y - quarter;// center point being center of our half the field
	int safeDistance = 460;
	bool fence = fakeY > quarter - safeDistance;
	bool back = -1*fakeY > quarter - safeDistance;
	bool right = fakeX > half - safeDistance;
	bool left = -1 * fakeX > half - safeDistance;


	//bool
	intakeAndLiftTask_intakeState = INTAKE_OPEN;
	intakeAndLiftTask_liftState = LIFT_DOWN;
	moveToPoint_Ballsy(x + (left-right) * safeDistance, y + (back - fence) * safeDistance, false, 0);
	moveToPoint_Ballsy(x, y, false, 315);
	return fence || back || right || left; // if was near any wall
}

#endif //MOTORCONTROL_C_INCLUDED
