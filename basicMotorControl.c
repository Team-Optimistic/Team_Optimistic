#ifndef BASICMOTORCONTROL_C_INCLUDED
#define BASICMOTORCONTROL_C_INCLUDED

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
	motor[driveLFY] = power;
	motor[driveLB] = power;
	motor[driveRFY] = power;
	motor[driveRB] = power;
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

#endif //BASICMOTORCONTROL_C_INCLUDED
