#ifndef SKILLS_C_INCLUDED
#define SKILLS_C_INCLUDED

void runSkills()
{
	startTask(readBuffer);
	wait1Msec(250);

	//PRELOAD AND STARS------------------------------------
  //Close claw and drive back
  intakeAndLiftTask_intakeState = INTAKE_REST; //Let the intake chill
  intakeAndLiftTask_liftState = LIFT_DOWN; //Put the lift down
  startTask(intakeAndLiftTask);

  //Drive back a tile and open the intake
  moveToPoint(609, 304 + ONE_TILE_MM*1.1, true);
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
  //moveToPoint(609, 304 + ONE_TILE_MM*1.2, true);
	writeDebugStreamLine("pre dump");
  //Dump
  dumpIntakeBasic();
	writeDebugStreamLine("post dump");
  //Turn to score star
  waitForLift(LIFT_DOWN);
  turnToAbsAngle(180);
  intakeAndLiftTask_intakeState = INTAKE_POPEN;
	writeDebugStreamLine("star");

  //CUBE MATCH LOADS------------------------
  //First cube
  //Drive back to wall at an offset
  moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM *0.7);
 	turnToAbsAngle(-90);

 	//Grab first cube and star
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(300);
 	driveStraight(-700); //Move intake out of way of wall
 	intakeAndLiftTask_intakeState = INTAKE_OPEN;
 	driveStraight(600);

 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(600);

 	driveStraight(-150);

 	//Drive back to fence
 	//driveStraight(-ONE_TILE_MM*0.5);

 	//Dump
 	dumpIntakeBasic();

 	//Second cube
 	//Drive back to wall
  moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM*1.2);

 	//Grab second cube
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(500);

 	//Drive back to fence
  moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM*1.1, true);

 	//Dump
 	dumpIntakeBasic();

  //CENTER CUBE------------------------------------
 	moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM);
 	intakeAndLiftTask_intakeState = INTAKE_HALF;
 	moveToPoint(609 + 0.9* ONE_TILE_MM,304 + ONE_TILE_MM);
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(250);
 	intakeAndLiftTask_liftState = LIFT_HALF;
 	moveToPoint(609 + 3.25* ONE_TILE_MM,304 + ONE_TILE_MM);
 	dumpIntakeBasic();
}

#endif //SKILLS_C_INCLUDED
