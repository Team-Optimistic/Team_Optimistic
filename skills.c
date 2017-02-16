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

  //Turn back
  wait1Msec(125);
  turn(90);

  //Close intake a raise lift so we can turn and not hit stars
  intakeAndLiftTask_intakeState = INTAKE_CLOSED;
  intakeAndLiftTask_liftState = LIFT_HALF;
  waitForLift(LIFT_HALF);

  //Turn to be parallel to fence
  turn(90);

  //Open intake halfway and lower lift
  intakeAndLiftTask_intakeState = INTAKE_HALF;
  waitForIntake(INTAKE_HALF);
  intakeAndLiftTask_liftState = LIFT_DOWN;
  waitForLift(LIFT_DOWN);

  //Drive to get 4 stars
  driveStraight(THREE_TILE_MM);

  //Close intake and lift
  intakeAndLiftTask_intakeState = INTAKE_CLOSED;
  waitForIntake(INTAKE_CLOSED);
  intakeAndLiftTask_liftState = LIFT_UP;

  //Drive back a bit and turn
  driveStraight(-ONE_TILE_MM / 2);
  turn(-90);

  //Dump stars and lower lift
  waitForLift(LIFT_UP);
  intakeAndLiftTask_intakeState = INTAKE_OPEN;
  wait1Msec(100);
  intakeAndLiftTask_liftState = LIFT_DOWN;
}

#endif //SKILLS_C_INCLUDED
