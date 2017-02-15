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
  while(intakeAndLiftTask_intakeStateRead != INTAKE_OPEN) { wait1Msec(5); }

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
  while(intakeAndLiftTask_liftStateRead != LIFT_UP){ wait1Msec(5); }

  //Open intake and lower lift
  intakeAndLiftTask_intakeState = INTAKE_OPEN;
  wait1Msec(100);
  intakeAndLiftTask_liftState = LIFT_DOWN;

  //Turn to score star
  while(intakeAndLiftTask_liftStateRead != LIFT_DOWN){wait1Msec(5);}
  turn(-90);
  wait1Msec(125);
  turn(90);

  //writeDebugStreamLine("drive to 100,0");
  //moveToPoint(FENCE_LEFT_X, FENCE_RIGHT_Y);

  //cheeseThoseStars(); //Get the two stars and preloads
  //dumpIntake(); //Score

  //moveToPoint(FENCE_RIGHT_X, FENCE_RIGHT_Y - 200); //Move a little forward
  //turnToAbsAngle(-90); //Face west

  //intakeAndLiftTask_intakeState = INTAKE_OPEN; //Open the intake
  //while(intakeAndLiftTask_intakeStateRead != INTAKE_OPEN) { wait1Msec(5); }

  //moveToPoint(FENCE_LEFT_X - 200, FENCE_RIGHT_Y - 200); //Move to the end

  //intakeAndLiftTask_intakeState = INTAKE_CLOSED; //Close the intake
  //while(intakeAndLiftTask_intakeStateRead != INTAKE_CLOSED) { wait1Msec(5); }

  //dumpIntake(); //Score

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
