#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    intakePot,      sensorPotentiometer)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftQuad,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightQuad,      sensorQuadEncoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define BCI_SEM_DEBUG_FULL
#define MENU_NUM 1
#define LCD_NO_CALLBACKS
#define LCD_NO_SAFETY
#include "BCI\BCI.h"

#include "uartHandler.c"
#include "motorControl.c"
#include "decisionHandler.c"

menu *pidReadout;

task main()
{
	pidReadout = newMenu("");

	driveStraight(100);

	//initUART();

	////Start reading from pi
	//startTask(readBuffer);

	////Let robot drive itself
	//startTask(commandRobot);

	//while (true)
	//{
	//	//Send data to pi
	//	sendSTDMsg();

	//	//Task wait
	//	wait1Msec(15);
	//}
}

task updateMenus()
{
	string msg;
	while (true)
	{
		sprintf(msg, "Theta: %d", SensorValue[leftQuad] - SensorValue[rightQuad]);
		changeMessage(pidReadout, msg);
	}
}
