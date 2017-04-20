/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: VEX Competition Control Include File                  */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*                                                                           */
/* This file provides control over a VEX Competition Match. It should be     */
/* included in the user's program with the following line located near the   */
/* start of the user's program.                                              */
/*                                                                           */
/* #include "VEX_Competition_Includes.h"                                     */
/*                                                                           */
/* The above statement will cause this program to be included in the user's  */
/* program. There's no need to modify this program.                          */
/*                                                                           */
/* The program displays status information on the new VEX LCD about the      */
/* competition state. You don't need the LCD, the program will work fine     */
/* whether or not the LCD is actually provisioned.                           */
/*                                                                           */
/* The status information is still useful without the LCD. The ROBOTC IDE    */
/* debugger has a "remote screen" that contains a copy of the status         */
/* information on the LCD. YOu can use this to get a view of the status of   */
/* your program. The remote screen is shown with the menu command            */
/*   "Robot -> Debugger Windows -> VEX Remote Screen"                        */
/*                                                                           */
/* The LCD is 2 lines x 16 characters. There are three display formats to    */
/* look for:                                                                 */
/*                                                                           */
/*        State          Description                                         */
/*                                                                           */
/*    ----------------                                                       */
/*   |Disabled        |  The robot is idle. This occurs before both the      */
/*   |Time mm:ss.s    |  autonomous and user control states. The time        */
/*    ----------------   display is minutes and seconds it has been idle.    */
/*                                                                           */
/*    ----------------                                                       */
/*   |Autonomous      |  The robot is running autonomous code.               */
/*   |Time mm:ss.s    |  control states. The time display is minutes and     */
/*    ----------------   seconds it has been running.                        */
/*                                                                           */
/*    ----------------                                                       */
/*   |User Control    |  The robot is running user control code.             */
/*   |Time mm:ss.s    |  control states. The time display is minutes and     */
/*    ----------------   seconds it has been running.                        */
/*---------------------------------------------------------------------------*/

// Forward references for functions in the competition template
void pre_auton();
task autonomous();
task usercontrol();

// This global controls whether this template stops tasks when the robot becomes
// disabled.
bool bStopTasksBetweenModes = true;

// This global is used to disable the status display on the LCD by the code
// in this file.  The user may want to use the LCD during the pre_auton
// function and not have it overwritten by calls to displayStatusAndTime
bool bDisplayCompetitionStatusOnLcd = true;

// There will also be many other errors but perhaps this will be noticed as it
// will be at the top of the list.  VEX platform is left for legacy reasons.
#if !defined(VEX2) && !defined(VEX)
#error "Switch to the VEX cortex platform"
#endif

// Forward references for functions in this file
static void displayStatusAndTime( bool reset = false );
       void UserControlCodePlaceholderForTesting();
       void AutonomousCodePlaceholderForTesting();
			 void allMotorsOff();
			 void allTasksStop();

/*---------------------------------------------------------------------------*/
task main()
{
	// Master CPU will not let competition start until powered on for at least 2-seconds
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDPos(0, 0);
	displayNextLCDString("Startup");
	wait1Msec(2000);
	clearLCDLine(0);

	// Call pre_auton function where the user can initialize sensors and variables
	pre_auton();

	while (true)
	{
		// reset timer
		displayStatusAndTime( true );

		// remain in this loop while the robot is disabled
		while (bIfiRobotDisabled)
		{
			int count = 0;
			while (true)
			{
				if (!bIfiRobotDisabled)
				  break;
				wait1Msec(25);
				//motor[port10] = 55;
				// Call every 100mS
				if(count++ == 3) {
					displayStatusAndTime();
					count = 0;
					}
			}
	  }

		// The robot has become enabled
		// Reset status timer
		displayStatusAndTime( true );

		if (bIfiAutonomousMode)
		{
			// Start the autonomous task
		  startTask(autonomous);

			// Waiting for autonomous phase to end
			while (bIfiAutonomousMode && !bIfiRobotDisabled)
			{
				if (!bVEXNETActive)
				{
				  if (nVexRCReceiveState == vrNoXmiters) // the transmitters are powered off!!
					  allMotorsOff();
				}
				wait1Msec(25);               // Waiting for autonomous phase to end
			}
		  allMotorsOff();
		  stopTask(commandRobot);
		}

		else
		{
			// Start the usercontrol task
			startTask(usercontrol);

			// Here we repeat loop waiting for user control to end and (optionally) start
			// of a new competition run
			while (!bIfiAutonomousMode && !bIfiRobotDisabled)
			{
				if (nVexRCReceiveState == vrNoXmiters) // the transmitters are powered off!!
					allMotorsOff();
				wait1Msec(25);
		  }
			allMotorsOff();
		  stopTask(commandRobot);
		}
	}

	// This code will never run
	// it stops the compiler from creating warnings about the placeholder functions
	// if they have been removed from the competition template.
	if(false) {
		UserControlCodePlaceholderForTesting();
		AutonomousCodePlaceholderForTesting();
		}
}

/*---------------------------------------------------------------------------*/
/* Function that will stop all motors                                        */
/*---------------------------------------------------------------------------*/
void allMotorsOff()
{
	motor[port1] = 0;
	motor[port2] = 0;
	motor[port3] = 0;
	motor[port4] = 0;
	motor[port5] = 0;
	motor[port6] = 0;
	motor[port7] = 0;
	motor[port8] = 0;
#if defined(VEX2)
	motor[port9] = 0;
	//motor[port10] = 0;
#endif
}

/*---------------------------------------------------------------------------*/
/* Function that will stop all tasks except the main task                    */
/*---------------------------------------------------------------------------*/
void allTasksStop()
{
  stopTask(1);
  stopTask(2);
  stopTask(3);
  stopTask(4);
#if defined(VEX2)
  stopTask(5);
  stopTask(6);
  stopTask(7);
  stopTask(8);
  stopTask(9);
  stopTask(10);
  stopTask(11);
  stopTask(12);
  stopTask(13);
  stopTask(14);
  stopTask(15);
  stopTask(16);
  stopTask(17);
  stopTask(18);
  stopTask(19);
#endif
}

/*---------------------------------------------------------------------------*/
/* This function is used to display the status and running time on the VEX   */
/* LCD display.                                                              */
/*---------------------------------------------------------------------------*/
static void displayStatusAndTime( bool reset )
{
	static int nDisplayAndStatusTimer = 0;

	if( !bDisplayCompetitionStatusOnLcd )
		return;

	if(reset) {
		nDisplayAndStatusTimer = 0;
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDPos(0, 0);
		return;
	}

  displayLCDPos(0, 0);
	if (bIfiRobotDisabled)
		displayNextLCDString("Disabled");

  displayLCDPos(1, 0);
	if (bIfiRobotDisabled)
	  displayNextLCDString("Disable ");
	else
	{
	  if (bIfiAutonomousMode)
	    displayNextLCDString("Auton  ");
	  else
	    displayNextLCDString("Driver ");
	}
	displayNextLCDNumber(nDisplayAndStatusTimer / 600, 2);
	displayNextLCDChar(':');
	displayNextLCDNumber((nDisplayAndStatusTimer / 10) % 60, -2);
	displayNextLCDChar('.');
	displayNextLCDNumber(nDisplayAndStatusTimer % 10, 1);
	++nDisplayAndStatusTimer;
}

/*---------------------------------------------------------------------------*/
/*  Placeholder function that is called from the competition template if     */
/*  the user has not modified the usercontrol task                           */
/*---------------------------------------------------------------------------*/
void UserControlCodePlaceholderForTesting()
{
  // Following code is simply for initial debuggging.
  //
  // It can be safely removed in a real program	and removing it will slightly
  // improve the real-time performance of your robot.
	//
  displayStatusAndTime();
	wait1Msec(100);
}

/*---------------------------------------------------------------------------*/
/*  Placeholder function that is called from the competition template if     */
/*  the user has not modified the autonomous task                            */
/*---------------------------------------------------------------------------*/
void AutonomousCodePlaceholderForTesting()
{
	// This is where you insert your autonomous code. Because we don't have any,
	// we'll simply display a running count of the time on the VEX LCD.

	while (true)
	{
	  displayStatusAndTime();
		wait1Msec(100);
	}
}
