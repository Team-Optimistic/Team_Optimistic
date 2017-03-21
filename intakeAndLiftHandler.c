#ifndef INTAKEANDLIFTHANDLER_C_INCLUDED
#define INTAKEANDLIFTHANDLER_C_INCLUDED

#define INTAKE_CLOSED_VAL  680
#define INTAKE_CUBE_VAL    800
#define INTAKE_OPEN_VAL    1400
#define INTAKE_POPEN_VAL   3627

#define LIFT_FENCE_VAL     1550
#define LIFT_UP_VAL        3000
#define LIFT_DUMP_VAL      2200
#define LIFT_HALF_VAL      1350
#define LIFT_DOWN_VAL      550

#define INTAKE_BANDWITH    20
#define LIFT_BANDWITH      30
#define LIFT_PID_BIAS      -10

long waitStartTime = 0;
#define waitForIntake(val) waitStartTime=nSysTime;while(intakeAndLiftTask_intakeStateRead != val){if(nSysTime-waitStartTime>1000){break;}wait1Msec(15);}
#define waitForLift(val) waitStartTime=nSysTime;while(intakeAndLiftTask_liftStateRead != val){if(nSysTime-waitStartTime>3000){break;}wait1Msec(15);}

enum intakeState
{
	INTAKE_CUBE    = 1,
	INTAKE_OPEN    = 3,
	INTAKE_POPEN   = 4,
	INTAKE_CLOSED  = 7,
	INTAKE_REST    = 8,
	INTAKE_WAIT    = 9
};

enum liftState
{
	LIFT_NOTHING = 0,
	LIFT_CUSTOM = 92,
	LIFT_UP     = 93,
	LIFT_DUMP   = 94,
	LIFT_HALF   = 95,
	LIFT_DOWN   = 96,
	LIFT_FENCE  = 97,
	LIFT_REST   = 98,
	LIFT_WAIT   = 99
};

intakeState intakeAndLiftTask_intakeState = INTAKE_REST;
intakeState intakeAndLiftTask_intakeStateRead = INTAKE_REST;

intakeState intakeAndLiftTask_liftState = LIFT_REST;
intakeState intakeAndLiftTask_liftStateRead = LIFT_REST;

float liftPosWithOffset = 0;
long intakeAndLiftTask_liftOffset = 0;
int intakeAndLiftTask_liftCustomVal = LIFT_DOWN_VAL;

/**
 * Maintains different states for the intake and lift
 */
task intakeAndLiftTask()
{
	bool liftHasGoneDownBefore = false;

	pos_PID intakePID, liftPID;

	pos_PID_InitController(&intakePID, intakePot, 0.3, 0.2, 0, 55);
	pos_PID_InitController(&liftPID, &liftPosWithOffset, 0.1, 0.2, 0.04, LIFT_PID_BIAS);

	while (true)
	{
		switch (intakeAndLiftTask_intakeState)
		{
			case INTAKE_CUBE:
				pos_PID_SetTargetPosition(&intakePID, INTAKE_CUBE_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_OPEN:
				pos_PID_SetTargetPosition(&intakePID, INTAKE_OPEN_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_POPEN:
				pos_PID_SetTargetPosition(&intakePID, INTAKE_POPEN_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_CLOSED:
				if (SensorValue[intakePot] <= 650)
				{
					setIntakeMotors(0);
				}
				else
				{
					pos_PID_SetTargetPosition(&intakePID, INTAKE_CLOSED_VAL);
					setIntakeMotors(pos_PID_StepController(&intakePID));
				}
				break;

			case INTAKE_REST:
				setIntakeMotors(0);
				break;

			case INTAKE_WAIT:
				break;
		}

		//This is where the intake actually is
		//INTAKE_CUBE
		if (SensorValue[intakePot] <= INTAKE_CUBE_VAL + INTAKE_BANDWITH &&
			  SensorValue[intakePot] >= INTAKE_CUBE_VAL - INTAKE_BANDWITH)
		{
			intakeAndLiftTask_intakeStateRead = INTAKE_CUBE;
		}
		//INTAKE_OPEN
		else if (SensorValue[intakePot] <= INTAKE_OPEN_VAL + INTAKE_BANDWITH &&
			  		 SensorValue[intakePot] >= INTAKE_OPEN_VAL - INTAKE_BANDWITH)
		{
			intakeAndLiftTask_intakeStateRead = INTAKE_OPEN;
		}
		//INTAKE_POPEN
		else if (SensorValue[intakePot] <= INTAKE_POPEN_VAL + INTAKE_BANDWITH &&
			  		 SensorValue[intakePot] >= INTAKE_POPEN_VAL - INTAKE_BANDWITH)
		{
			intakeAndLiftTask_intakeStateRead = INTAKE_POPEN;
		}
		//INTAKE_CLOSED
		else if (SensorValue[intakePot] <= INTAKE_CLOSED_VAL + INTAKE_BANDWITH &&
		         SensorValue[intakePot] >= INTAKE_CLOSED_VAL - INTAKE_BANDWITH)
	  {
			intakeAndLiftTask_intakeStateRead = INTAKE_CLOSED;
	  }

	 	//Reset offset if we hit the bottom
		if (SensorValue[liftStopButton])
		{
			intakeAndLiftTask_liftOffset = SensorValue[liftPot];
			liftHasGoneDownBefore = true;
		}

		//Update count with offset
		liftPosWithOffset = 4096 - SensorValue[liftPot];

		switch (intakeAndLiftTask_liftState)
		{
			case LIFT_CUSTOM:
				pos_PID_SetTargetPosition(&liftPID, intakeAndLiftTask_liftCustomVal);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_UP:
				pos_PID_SetTargetPosition(&liftPID, LIFT_UP_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_DUMP:
				pos_PID_SetTargetPosition(&liftPID, LIFT_DUMP_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_HALF:
				pos_PID_SetTargetPosition(&liftPID, LIFT_HALF_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_DOWN:
				if (!SensorValue[liftStopButton])
				{
					setLiftMotors(-127);
				}
				else
				{
					setLiftMotors(LIFT_PID_BIAS);
				}
				break;

			case LIFT_FENCE:
				pos_PID_SetTargetPosition(&liftPID, LIFT_FENCE_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_REST:
				setLiftMotors(0);
				break;

			case LIFT_WAIT:
				break;
		}

		//This is where the lift actually is
		//LIFT_UP
		if (liftPosWithOffset <= LIFT_UP_VAL + LIFT_BANDWITH &&
		    liftPosWithOffset >= LIFT_UP_VAL - LIFT_BANDWITH)
		{
			intakeAndLiftTask_liftStateRead = LIFT_UP;
		}
		//LIFT_DUMP
		else if (liftPosWithOffset <= LIFT_DUMP_VAL + LIFT_BANDWITH &&
			       liftPosWithOffset >= LIFT_DUMP_VAL - LIFT_BANDWITH)
    {
    	intakeAndLiftTask_liftStateRead = LIFT_DUMP;
    }
		//LIFT_HALF
		else if (liftPosWithOffset <= LIFT_HALF_VAL + LIFT_BANDWITH &&
			       liftPosWithOffset >= LIFT_HALF_VAL - LIFT_BANDWITH)
		{
		  intakeAndLiftTask_liftStateRead = LIFT_HALF;
		}
		//LIFT_DOWN
		else if (SensorValue[liftStopButton])
		{
			intakeAndLiftTask_liftStateRead = LIFT_DOWN;
		}
		else
		{
			intakeAndLiftTask_liftStateRead = LIFT_NOTHING;
		}

		wait1Msec(15);
	}
}

#endif //INTAKEANDLIFTHANDLER_C_INCLUDED
