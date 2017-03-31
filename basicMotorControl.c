#ifndef BASICMOTORCONTROL_C_INCLUDED
#define BASICMOTORCONTROL_C_INCLUDED

const int max_pwr = 70;
void setLeftMotors(int powerValue)
{
	if(abs(powerValue)>max_pwr)
		powerValue = max_pwr * sgn(powerValue);
	motor[driveLFY] = powerValue;
	motor[driveLB] = powerValue;
}

void setRightMotors(int powerValue)
{
	if(abs(powerValue)>max_pwr)
		powerValue = max_pwr * sgn(powerValue);
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
