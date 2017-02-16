#ifndef INTAKEANDLIFTHANDLER_C_INCLUDED
#define INTAKEANDLIFTHANDLER_C_INCLUDED

#define INTAKE_OPEN_VAL   1020
#define INTAKE_HALF_VAL   2000
#define INTAKE_CLOSED_VAL 2400
#define INTAKE_BANDWITH   100

#define LIFT_UP_VAL       1350
#define LIFT_HALF_VAL     500
#define LIFT_DOWN_VAL     0
#define LIFT_FENCE_VAL    1540
#define LIFT_BANDWITH     30

#define waitForIntake(val) while(intakeAndLiftTask_intakeStateRead != val){wait1Msec(5);}
#define waitForLift(val) while(intakeAndLiftTask_liftStateRead != val){wait1Msec(5);}

enum intakeState
{
	INTAKE_OPEN,
	INTAKE_HALF,
	INTAKE_CLOSED,
	INTAKE_REST,
	INTAKE_WAIT
};

enum liftState
{
	LIFT_UP,
	LIFT_HALF,
	LIFT_DOWN,
	LIFT_FENCE,
	LIFT_REST,
	LIFT_WAIT
};

intakeState intakeAndLiftTask_intakeState = INTAKE_REST;
intakeState intakeAndLiftTask_intakeStateRead = INTAKE_REST;

intakeState intakeAndLiftTask_liftState = LIFT_REST;
intakeState intakeAndLiftTask_liftStateRead = LIFT_REST;

long intakeAndLiftTask_imeOffset = 0;

/**
 * Maintains different states for the intake and lift
 */
task intakeAndLiftTask()
{
	float imeCountWithOffset = 0;
	pos_PID intakePID, liftPID;

	pos_PID_InitController(&intakePID, intakePot, 0.15, 0.2, 0, 0);
	pos_PID_InitController(&liftPID, &imeCountWithOffset, 0.3, 0.2, 0.1, -10);

	while (true)
	{
		switch (intakeAndLiftTask_intakeState)
		{
			case INTAKE_OPEN:
				pos_PID_ChangeBias(&intakePID, 0);
				pos_PID_SetTargetPosition(&intakePID, INTAKE_OPEN_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_HALF:
				pos_PID_ChangeBias(&intakePID, 0);
				pos_PID_SetTargetPosition(&intakePID, INTAKE_HALF_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_CLOSED:
				if (SensorValue[intakePot] >= 2320)
				{
					setIntakeMotors(0);
				}
				else
				{
					pos_PID_ChangeBias(&intakePID, -30);
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
		if (SensorValue[intakePot] <= INTAKE_OPEN_VAL + INTAKE_BANDWITH &&
			  SensorValue[intakePot] >= INTAKE_OPEN_VAL - INTAKE_BANDWITH)
		{
			intakeAndLiftTask_intakeStateRead = INTAKE_OPEN;
		}
		else if (SensorValue[intakePot] <= INTAKE_HALF_VAL + INTAKE_BANDWITH &&
			  		 SensorValue[intakePot] >= INTAKE_HALF_VAL - INTAKE_BANDWITH)
		{
			intakeAndLiftTask_intakeStateRead = INTAKE_HALF;
		}
		else if (SensorValue[intakePot] <= INTAKE_CLOSED_VAL + INTAKE_BANDWITH &&
		         SensorValue[intakePot] >= INTAKE_CLOSED_VAL - INTAKE_BANDWITH)
	  {
			intakeAndLiftTask_intakeStateRead = INTAKE_CLOSED;
	  }

	 	//Reset offset if we hit the bottom
		if (SensorValue[liftStopButton])
		{
			intakeAndLiftTask_imeOffset = nMotorEncoder[liftRI];
		}

		//Update count with offset
		imeCountWithOffset = nMotorEncoder[liftRI] - intakeAndLiftTask_imeOffset;

		switch (intakeAndLiftTask_liftState)
		{
			case LIFT_UP:
				pos_PID_SetTargetPosition(&liftPID, LIFT_UP_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_HALF_VAL:
				pos_PID_SetTargetPosition(&liftPID, LIFT_HALF_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_DOWN:
				if (!SensorValue[liftStopButton])
				{
					setLiftMotors(-100);
				}
				else
				{
					pos_PID_SetTargetPosition(&liftPID, LIFT_DOWN_VAL);
					setLiftMotors(pos_PID_StepController(&liftPID));
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
		if (imeCountWithOffset <= LIFT_UP_VAL + LIFT_BANDWITH &&
		    imeCountWithOffset >= LIFT_UP_VAL - LIFT_BANDWITH)
		{
			intakeAndLiftTask_liftStateRead = LIFT_UP;
		}
		else if (imeCountWithOffset <= LIFT_DOWN_VAL + LIFT_BANDWITH)
		{
			intakeAndLiftTask_liftStateRead = LIFT_DOWN;
		}

		wait1Msec(15);
	}
}

#endif //INTAKEANDLIFTHANDLER_C_INCLUDED
