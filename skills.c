#ifndef SKILLS_C_INCLUDED
#define SKILLS_C_INCLUDED

void runSkills()
{
  //Close claw and drive back
  intakeAndLiftTask_intakeState = INTAKE_REST; //Let the intake chill
  intakeAndLiftTask_liftState = LIFT_DOWN; //Put the lift down
  startTask(intakeAndLiftTask);

  intakeAndLiftTask_intakeState = INTAKE_OPEN;
  cheeseThoseStars();

  // driveStraight(-610); //Drive back off the tile
  // intakeAndLiftTask_intakeState = INTAKE_CLOSED; //Close the intake
  //
  // //Wait until intake is closed
  // while (intakeAndLiftTask_intakeStateRead != INTAKE_CLOSED) { wait1Msec(5); }
  //
  // dumpIntake(); //Score what we picked up from the starting tile
  //
  // scoreFence(FENCE_RIGHT); //Knock the stars off the rightmost fence segment
  //
  // moveToPoint();
}

#endif //SKILLS_C_INCLUDED
