#ifndef SKILLS_C_INCLUDED
#define SKILLS_C_INCLUDED

void runSKills()
{
  //Close claw and drive back
  intakeAndLiftTask_intakeState = INTAKE_CLOSED;
  intakeAndLiftTask_liftState = LIFT_DOWN;
  startTask(intakeAndLiftTask);
  driveStraight(-610);
}

#endif //SKILLS_C_INCLUDED
