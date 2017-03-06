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
  moveToPoint(609, 304 + ONE_TILE_MM*1.4, true);

  //Dump
  dumpIntakeBasic();

  //Turn to score star
  waitForLift(LIFT_DOWN);
  intakeAndLiftTask_liftCustomVal = 50;
  intakeAndLiftTask_liftState = LIFT_CUSTOM;
  turnToAbsAngle(270);
  intakeAndLiftTask_liftState = LIFT_DOWN;

  //CUBE MATCH LOADS------------------------
  //First cube
  //Drive back to wall at an offset
  moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM);
 	turnToAbsAngle(180);

 	//Grab first cube and star
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(100);
 	driveStraight(-200); //Move intake out of way of wall
 	intakeAndLiftTask_intakeState = INTAKE_OPEN;
 	setAllDriveMotors(127);
 	wait1Msec(150);
 	setAllDriveMotors(0);
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(100);
 	setAllDriveMotors(-127);
 	wait1Msec(150);
 	setAllDriveMotors(0);
 	intakeAndLiftTask_liftState = LIFT_UP;

 	//Drive back to fence
 	//moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM*1.4, true);
 	driveStraight(-ONE_TILE_MM*0.5);

 	//Dump
 	turnToAbsAngle(180);
 	dumpIntakeBasic();

 	//Second cube
 	//Drive back to wall
  moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM*1.1);
 	turnToAbsAngle(180);

 	//Grab second cube
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	wait1Msec(250);
 	intakeAndLiftTask_liftState = LIFT_UP;

 	//Drive back to fence
  moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM*1.4, true);

 	//Dump
  turnToAbsAngle(180);
 	dumpIntakeBasic();

  //CENTER CUBE------------------------------------
 	moveToPoint(609 - ONE_TILE_MM/4, 304 + ONE_TILE_MM*1.1);
}

#endif //SKILLS_C_INCLUDED
