#ifndef INTAKEANDLIFTHANDLER_C_INCLUDED
#define INTAKEANDLIFTHANDLER_C_INCLUDED

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
intakeState intakeAndLiftTask_liftState = LIFT_REST;

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
				pos_PID_SetTargetPosition(&intakePID, 1020);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_CLOSED:
				pos_PID_ChangeBias(&intakePID, -30);
				pos_PID_SetTargetPosition(&intakePID, 2220);
				setIntakeMotors(pos_PID_StepController(&intakePID));
				break;

			case INTAKE_REST:
				setIntakeMotors(0);
				break;

			case INTAKE_WAIT:
				break;
		}

		if (SensorValue[liftStopButton])
		{
			nMotorEncoder[liftRI] = 0;
		}

		switch (intakeAndLiftTask_liftState)
		{
			case LIFT_UP:
				pos_PID_SetTargetPosition(&liftPID, 1350); //Up position
				setLiftMotors(pos_PID_StepController(&liftPID));
				break;

			case LIFT_DOWN:
				pos_PID_SetTargetPosition(&liftPID, 0);
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
}

#endif //INTAKEANDLIFTHANDLER_C_INCLUDED
