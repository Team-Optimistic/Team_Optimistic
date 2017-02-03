#ifndef INTAKEANDLIFTHANDLER_C_INCLUDED
#define INTAKEANDLIFTHANDLER_C_INCLUDED

#define INTAKE_OPEN_VAL   1020
#define INTAKE_CLOSED_VAL 2220
#define INTAKE_BANDWITH   100

#define LIFT_UP_VAL       1350
#define LIFT_DOWN_VAL     0

enum intakeState
{
	INTAKE_OPEN,
	INTAKE_CLOSED,
	INTAKE_REST,
	INTAKE_WAIT
};

enum liftState
{
	LIFT_UP,
	LIFT_DOWN,
	LIFT_REST,
	LIFT_WAIT
};

intakeState intakeAndLiftTask_intakeState = INTAKE_REST;
intakeState intakeAndLiftTask_intakeStateRead = INTAKE_REST;

intakeState intakeAndLiftTask_liftState = LIFT_REST;
intakeState intakeAndLiftTask_liftStateRead = LIFT_REST;

/**
 * Maintains different states for the intake and lift
 */
task intakeAndLiftTask()
{
	pos_PID intakePID, liftPID;

	pos_PID_InitController(&intakePID, intakePot, 0.2, 0.1, 0, 0);
	pos_PID_InitController(&liftPID, liftRI, 0.3, 0.2, 0.04, -10);

	while (true)
	{
		switch (intakeAndLiftTask_intakeState)
		{
			case INTAKE_OPEN:
				pos_PID_ChangeBias(&intakePID, 0);
				pos_PID_SetTargetPosition(&intakePID, INTAKE_OPEN_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_CLOSED:
				pos_PID_ChangeBias(&intakePID, -30);
				pos_PID_SetTargetPosition(&intakePID, INTAKE_CLOSED_VAL);
				setIntakeMotors(pos_PID_StepController(&intakePID));
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
		else if (SensorValue[intakePot] <= INTAKE_CLOSED_VAL + INTAKE_BANDWITH &&
		         SensorValue[intakePot] >= INTAKE_CLOSED_VAL - INTAKE_BANDWITH)
	  {
			intakeAndLiftTask_intakeStateRead = INTAKE_CLOSED;
	  }

		if (SensorValue[liftStopButton])
		{
			nMotorEncoder[liftRI] = 0;
		}

		switch (intakeAndLiftTask_liftState)
		{
			case LIFT_UP:
				pos_PID_SetTargetPosition(&liftPID, LIFT_UP_VAL); //Up position
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_DOWN:
				pos_PID_SetTargetPosition(&liftPID, LIFT_DOWN_VAL);
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_REST:
				setLiftMotors(0);
				break;

			case LIFT_WAIT:
				break;
		}

		wait1Msec(15);
	}

	//This is where the lift actually is
	if (nMotorEncoder[liftRI] <= LIFT_UP_VAL + 5 &&
	    nMotorEncoder[liftRI] >= LIFT_UP_VAL - 5)
	{
		intakeAndLiftTask_liftStateRead = LIFT_UP;
	}
	else if (nMotorEncoder[liftRI] <= LIFT_DOWN_VAL + 5 &&
	         nMotorEncoder[liftRI] >= LIFT_DOWN_VAL - 5)
	{
		intakeAndLiftTask_liftStateRead = LIFT_DOWN;
	}
}

#endif //INTAKEANDLIFTHANDLER_C_INCLUDED
