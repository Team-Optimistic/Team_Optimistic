#ifndef SKILLS_C_INCLUDED
#define SKILLS_C_INCLUDED

void runSkills()
{
  //Close claw and drive back
	writeDebugStreamLine("close claw and drive back");
  intakeAndLiftTask_intakeState = INTAKE_REST; //Let the intake chill
  intakeAndLiftTask_liftState = LIFT_DOWN; //Put the lift down
  startTask(intakeAndLiftTask);

  writeDebugStreamLine("open the intake");
  intakeAndLiftTask_intakeState = INTAKE_OPEN; //Open the intake
  while(intakeAndLiftTask_intakeStateRead != INTAKE_OPEN) { wait1Msec(5); }

  writeDebugStreamLine("drive to 100,0");
  moveToPoint(FENCE_LEFT_X, FENCE_RIGHT_Y);

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
