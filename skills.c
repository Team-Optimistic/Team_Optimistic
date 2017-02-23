#ifndef SKILLS_C_INCLUDED
#define SKILLS_C_INCLUDED

void runSkills()
{
	//PRELOAD AND STARS------------------------------------
  //Close claw and drive back
  intakeAndLiftTask_intakeState = INTAKE_REST; //Let the intake chill
  intakeAndLiftTask_liftState = LIFT_DOWN; //Put the lift down
  startTask(intakeAndLiftTask);

  //Drive back a tile and open the intake
  moveToPoint_Translate(0, ONE_TILE_MM, true);
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
  moveToPoint_Translate(0, ONE_TILE_MM*0.8, true);

  //Dump
  dumpIntakeBasic();

  //Turn to score star
  waitForLift(LIFT_DOWN);
  turnToAbsAngle(270);

  //CUBE MATCH LOADS------------------------
  //First cube
  //Drive back at an offset
  turnToAbsAngle(200);
  moveToPoint_Translate(0, -ONE_TILE_MM*0.8);
 	turnToAbsAngle(180);

 	//Grab first cube and star
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	moveToPoint_Translate(0, 200, true); //Move intake out of way of field wall while it closes
 	intakeAndLiftTask_liftState = LIFT_UP;
 	wait1Msec(100);

 	//Drive back to fence
 	moveToPoint_Translate(0, ONE_TILE_MM*0.8, true);

 	//Dump
 	turnToAbsAngle(180);
 	dumpIntakeBasic();

 	//Second cube
 	//Drive back
 	turnToAbsAngle(180);
 	moveToPoint_Translate(0, -ONE_TILE_MM*0.8);
 	turnToAbsAngle(180);

 	//Grab second cube
 	intakeAndLiftTask_intakeState = INTAKE_CLOSED;
 	intakeAndLiftTask_liftState = LIFT_UP;
 	wait1Msec(250);

 	//Drive back to fence
 	moveToPoint_Translate(0, ONE_TILE_MM*0.8, true);

 	//Dump
 	dumpIntakeBasic();

  ////CENTER CUBE------------------------------------
  ////Drive forward a bit to align with the cube
  //driveStraight((ONE_TILE_MM*0.8));

  ////Open intake far to avoid stars
  //intakeAndLiftTask_intakeState = INTAKE_POPEN;

  ////Turn to face cube
  //turnToAbsAngle(90);

  ////Close intake to avoid stars
  //intakeAndLiftTask_intakeState = INTAKE_ACUBE;
  //waitForIntake(INTAKE_OPEN);

  ////Drive onto cube
  //driveStraight(ONE_TILE_MM);

  ////Close intake around cube
  //intakeAndLiftTask_intakeState = INTAKE_CUBE;

  ////Raise lift a little
  //intakeAndLiftTask_liftState = LIFT_HALF;
  //wait1Msec(100);

  ////Drive forward a bit to be in the middle of the middle fence segment
  //driveStraight(ONE_TILE_MM*0.5);

  ////Turn to field wall
  //turnToAbsAngle(-90);

  ////Drive back and score
  //driveStraight(-ONE_TILE_MM*0.5);
  //intakeAndLiftTask_liftState = LIFT_UP;
  //waitForLift(LIFT_UP);
  //intakeAndLiftTask_intakeState = INTAKE_OPEN;
  //wait1Msec(100);
  //intakeAndLiftTask_liftState = LIFT_DOWN;

  ////STARS-------------------------------
  ////Drive to be closer to stars
  //waitForLift(LIFT_DOWN);
  //driveStraight(ONE_TILE_MM);

  ////Close intake a drive back
  //intakeAndLiftTask_intakeState = INTAKE_CLOSED;
  //driveStraight(-ONE_TILE_MM*1.5);

  ////Dump stars
  //intakeAndLiftTask_liftState = LIFT_UP;
  //waitForLift(LIFT_UP);
  //intakeAndLiftTask_intakeState = INTAKE_OPEN;
  //wait1Msec(100);
  //intakeAndLiftTask_liftState = LIFT_DOWN;
}

#endif //SKILLS_C_INCLUDED
