#ifndef SKILLS_C_INCLUDED
#define SKILLS_C_INCLUDED

void runSkills()
{
  //Close claw and drive back
  intakeAndLiftTask_intakeState = INTAKE_REST; //Let the intake chill
  intakeAndLiftTask_liftState = LIFT_DOWN; //Put the lift down
  startTask(intakeAndLiftTask);

  //Drive back a tile and open the intake
  driveStraight(-ONE_TILE_MM);
  intakeAndLiftTask_intakeState = INTAKE_OPEN; //Open the intake
  waitForIntake(INTAKE_OPEN);

  //Wait for match loads
  wait1Msec(1000);

  //Close the intake
  intakeAndLiftTask_intakeState = INTAKE_CLOSED;
  wait1Msec(500);

  //Drive back and lift to hold stars
  intakeAndLiftTask_liftState = LIFT_UP;
  wait1Msec(250);
  driveStraight(-ONE_TILE_MM*0.8);
  wait1Msec(500);

  //Lift up
  waitForLift(LIFT_UP);

  //Dump stars and lower lift
  intakeAndLiftTask_intakeState = INTAKE_OPEN;
  wait1Msec(100);
  intakeAndLiftTask_liftState = LIFT_DOWN;

  //Turn to score star
  waitForLift(LIFT_DOWN);
  turn(-90);
  turn(90);

  //Drive forward a bit to align with the cube
  driveStraight((ONE_TILE_MM*0.8));

  //Open intake far to avoid cube
  intakeAndLiftTask_intakeState = INTAKE_POPEN;

  //Turn to face cube
  turn(90);

  //Lightly close intake around cube
  intakeAndLiftTask_intakeState = INTAKE_QUARTER;
  waitForIntake(INTAKE_QUARTER);

  //Drive forward a bit to be in the middle of the middle fence segment
  driveStraight(ONE_TILE_MM * 1.5);

  //Turn and score cube
}

#endif //SKILLS_C_INCLUDED
