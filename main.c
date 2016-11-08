#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartUserControl, baudRate115200, IOPins, None, None)
#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in8,    intakePot,      sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftQuad,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightQuad,      sensorQuadEncoder)
#pragma config(Sensor, dgtl11, liftStopButton, sensorTouch)
#pragma config(Sensor, I2C_1,  liftIME,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           rightDrive11,  tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           rightDrive22,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           rightDrive21,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           rightDrive12,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           leftDrive12,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           leftDrive11,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           leftDrive2122, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           intakeMotors,  tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           liftMotors,    tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port10,          lidar,         tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define BCI_SEM_DEBUG_FULL_OFF
#define MENU_NUM 1
#define LCD_NO_CALLBACKS
#define LCD_NO_SAFETY
#include "BCI\BCI.h"

#define UARTHANDLER_DEBUG
#include "uartHandler.c"
#include "motorControl.c"
#include "decisionHandler.c"

task main()
{
	clearDebugStream();

	motor[lidar] = 64; //7.51V

	configureSerialPort(UART1, uartVEXLCD);
	setBaudRate(UART1, baudRate19200);

	newMenu("Test Menu");

	startTask(updateLCDTask);

	//initUART();

	//Start reading from pi
	//startTask(readBuffer);

	// //Let robot drive itself
	// startTask(commandRobot);

	while (true)
	{
		//Send data to pi
		//sendSTDMsg();

		//Temporary driver cotrol
		setLeftMotors(vexRT[JOY_JOY_LV]);
		setRightMotors(vexRT[JOY_JOY_RV]);
		//setIntakeMotors(127 * vexRT[JOY_TRIG_LU] + -127 * vexRT[JOY_TRIG_LD]);
		//setLiftMotors(127 * vexRT[JOY_TRIG_RU] + -127 * vexRT[JOY_TRIG_RD]);

		//Task wait
		wait1Msec(50);
	}
}
