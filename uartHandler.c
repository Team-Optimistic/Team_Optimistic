#ifndef UARTHANDLER_H_INCLUDED
#define UARTHANDLER_H_INCLUDED

/*
	Message header is
	<start byte 0xFA>
	<msg type byte>
	<short msg count>
*/

//-----------------------------------------------------------------------------

/*
	STD msg send structure is
	<header>
	<short intake pot val>
	<short left quad val>
	<short right quad val>
	<short dt>

	STD msg recieve structure is
	<header>
	<short estimated x>
	<short estimated y>
	<short estimated theta>
	<short lidar rpm>
*/

 //-----------------------------------------------------------------------------

/*
	MPC msg send structure is
	<header>

	MPC msg recieve structure is
	<header>
	<short x coordinate demand>
	<short y coordinate demand>
	<short pick up object> // 0 = do not pick up
												 // 1 = pick up star
												 // 2 = pick up cube
 	<short x coordinate demand>
 	<short y coordinate demand>
 	<short pick up object> // 0 = do not pick up
 												 // 1 = pick up star
 												 // 2 = pick up cube
	<short x coordinate demand>
	<short y coordinate demand>
	<short pick up object> // 0 = do not pick up
												 // 1 = pick up star
												 // 2 = pick up cube
 */

//Standard message
#define STD_MSG_LENGTH 13
long std_msg[STD_MSG_LENGTH];
#define STD_MSG_EST_X     0
#define STD_MSG_EST_Y     4
#define STD_MSG_EST_THETA 8
#define STD_MSG_LIDAR_RPM 12

//Message to pick up an object
#define MPC_MSG_LENGTH 36
long mpc_msg[MPC_MSG_LENGTH];
bool mpcMsgFlag = false;
#define MPC_MSG_X_COORD   0
#define MPC_MSG_Y_COORD   4
#define MPC_MSG_PICKUP    8
#define MPC_MSG_PICKUP_CLEAR 0
#define MPC_MSG_PICKUP_STAR  1
#define MPC_MSG_PICKUP_CUBE  2
#define MPC_MSG_PICKUP_BACK  3
#define MPC_MSG_PICKUP_WALL  4
#define MPC_MSG_OBJ_COUNT    3

//Message write semaphore, always get lock before reading
//Only task readBuffer() may write to msg
TSemaphore std_msgSem, mpc_msgSem;

//General UART semaphore, always get lock before writing
//Only task readBuffer() may read from UART
TSemaphore uartSem;

//Message count
#define MSG_COUNT_LENGTH 3
short msgCount[MSG_COUNT_LENGTH];
#define MSG_COUNT_TOTAL 0
#define MSG_COUNT_STD   1
#define MSG_COUNT_MPC   2

//Message types
#define STD_MSG_TYPE 1
#define MPC_MSG_TYPE 2

/*
Initializes everything this file needs for comms with the pi
 */
void initUART()
{
	//Initialize counts
	for (unsigned int i = 0; i < MSG_COUNT_LENGTH; i++)
		msgCount[i] = 0;

	std_msg[STD_MSG_EST_X] = 609;
	std_msg[STD_MSG_EST_Y] = 304;
	std_msg[STD_MSG_EST_THETA] = -90;
	std_msg[STD_MSG_LIDAR_RPM] = 125;

	//Initialize semaphores
	semaphoreInitialize(std_msgSem);
	semaphoreInitialize(mpc_msgSem);
	semaphoreInitialize(uartSem);

	//Setup UART1
	configureSerialPort(UART1, uartUserControl);
	setBaudRate(UART1, baudRate115200);

	//Purge UART1
	BCI_UART_ClearDataInBuffer(UART1);
}

/*
Generates new message count without side-effects
@param type Message type
 */
short uart_getMessageCount_Soft(const short type)
{
	if (type > 2)
	{
		writeDebugStreamLine("UART Handler: GetMsgCountSoft: Bad type: %d", type);
		return 0;
	}

	return msgCount[type] + 1 >= 255 ? 0 : msgCount[type] + 1;
}

/*
Generates new message count
@param type Message type
 */
short uart_getMessageCount(const short type)
{
	if (type > 2)
	{
		writeDebugStreamLine("UART Handler: GetMsgCount: Bad type: %d", type);
		return 0;
	}

	return msgCount[type] + 1 >= 255 ? (msgCount[type] = 0) : (msgCount[type] = msgCount[type] + 1);
}

/*
Verifies message count
@param type Message type
@param count New message count
 */
bool uart_verifyMessageCount(const short type, const short count)
{
	//Check if new count is what's expected
	const bool isGood = count == uart_getMessageCount_Soft(type);
	if (isGood)
	{
		msgCount[type] = count;
		return true;
	}

	return false;
}

/*
Sends a message header
@param type Message type
 */
void uart_sendMessageHeader(const short type)
{
	//Send start byte
	sendChar(UART1, 0xFA);

	//Send msg type
	sendChar(UART1, type);

	//Send msg count
	sendChar(UART1, uart_getMessageCount(type));

	#ifdef UARTHANDLER_DEBUG
		writeDebugStreamLine("UART Handler: Sent message header: 0xFA,%d,%d", type, msgCount[type]);
	#endif
}

union long2Bytes { long l; char b[sizeof(long)]; };
long2Bytes stdMsgUnion;
/*
Sends a standard message to the pi
 */
long sendSTDMsg_dt_last = nSysTime;
void sendSTDMsg()
{
	BCI_lockSem(uartSem, "sendSTDMsg")
	{
		//Send header
		uart_sendMessageHeader(STD_MSG_TYPE);

		//Send analog data
		sendChar(UART1, SensorValue[intakePot]);

		//Send digital data
		stdMsgUnion.l = nMotorEncoder[driveLFY];
		sendChar(UART1, stdMsgUnion.b[0]);
		sendChar(UART1, stdMsgUnion.b[1]);
		sendChar(UART1, stdMsgUnion.b[2]);
		sendChar(UART1, stdMsgUnion.b[3]);

		stdMsgUnion.l = nMotorEncoder[driveRFY];
		sendChar(UART1, stdMsgUnion.b[0]);
		sendChar(UART1, stdMsgUnion.b[1]);
		sendChar(UART1, stdMsgUnion.b[2]);
		sendChar(UART1, stdMsgUnion.b[3]);
	 	char dt = (char)(nSysTime - sendSTDMsg_dt_last);
		sendChar(UART1, dt);
		sendSTDMsg_dt_last = nSysTime;

		while (!bXmitComplete(UART1)) { wait1Msec(1); }

		#ifdef UARTHANDLER_DEBUG
			writeDebugStreamLine("UART Handler: Sent STD msg");
		#endif

		BCI_unlockSem(uartSem, "sendSTDMsg")
	}
}

/*
Tells the pi what object we just picked up
 */
void sendMPCMsg()
{
	BCI_lockSem(uartSem, "sendMPCMsg")
	{
		uart_sendMessageHeader(MPC_MSG_TYPE);

		while (!bXmitComplete(UART1)) { wait1Msec(1); }

		#ifdef UARTHANDLER_DEBUG
			writeDebugStreamLine("UART Handler: Sent MPC msg");
		#endif

		BCI_unlockSem(uartSem, "sendMPCMsg")
	}
}

/*
Reads in a message
 */
void uart_readMsg(long *msg, const unsigned int length)
{
	#ifdef UARTHANDLER_DEBUG_READ
		writeDebugStream("uart: read: ");
	#endif

	for (unsigned int index = 0; index < length; index++)
	{
		BCI_UART_ReadNextData(msg[index], UART1);

		#ifdef UARTHANDLER_DEBUG_READ
			writeDebugStream("%d,", msg[index]);
		#endif
	}

	#ifdef UARTHANDLER_DEBUG_READ
		writeDebugStreamLine("");
	#endif
}

/*
Polls uart for a message and records it into msg[]
 */
task readBuffer()
{
	short msgFlagHolder, msgTypeFlagHolder, msgCountFlagHolder;
	timer lidarTimer;
	timer_Initialize(&lidarTimer);

	long2Bytes conv;

	while (true)
	{
		//Send data to pi
		sendSTDMsg();

		//Load start byte into temp variable
		msgFlagHolder = getChar(UART1);

		if (msgFlagHolder == 0xFA)
		{
			//Get msg type
			BCI_UART_ReadNextData(msgTypeFlagHolder, UART1);

			//Get msg count
			BCI_UART_ReadNextData(msgCountFlagHolder, UART1);

			#ifdef UARTHANDLER_DEBUG
				//Verify msg count
				if (!uart_verifyMessageCount(msgTypeFlagHolder, msgCountFlagHolder))
				{
						writeDebugStreamLine("UART Handler: ReadBuffer: Bad msg count: %d", msgCountFlagHolder);
				}
			#endif

			switch (msgTypeFlagHolder)
			{
				case STD_MSG_TYPE:
					//Read in std msg
					BCI_lockSem(std_msgSem, "readBuffer")
					{
						#ifdef UARTHANDLER_DEBUG
							writeDebugStream("STD Msg: ");
						#endif
						uart_readMsg(std_msg, STD_MSG_LENGTH);

						conv.b[0] = std_msg[STD_MSG_EST_X];
						conv.b[1] = std_msg[STD_MSG_EST_X + 1];
						conv.b[2] = std_msg[STD_MSG_EST_X + 2];
						conv.b[3] = std_msg[STD_MSG_EST_X + 3];
						std_msg[STD_MSG_EST_X] = conv.l;

						conv.b[0] = std_msg[STD_MSG_EST_Y];
						conv.b[1] = std_msg[STD_MSG_EST_Y + 1];
						conv.b[2] = std_msg[STD_MSG_EST_Y + 2];
						conv.b[3] = std_msg[STD_MSG_EST_Y + 3];
						std_msg[STD_MSG_EST_Y] = conv.l;

						conv.b[0] = std_msg[STD_MSG_EST_THETA];
						conv.b[1] = std_msg[STD_MSG_EST_THETA + 1];
						conv.b[2] = std_msg[STD_MSG_EST_THETA + 2];
						conv.b[3] = std_msg[STD_MSG_EST_THETA + 3];
						std_msg[STD_MSG_EST_THETA] = conv.l;

						#ifdef UARTHANDLER_DEBUG_READ
							writeDebugStreamLine("decoded: %d, %d, %d", std_msg[STD_MSG_EST_X], std_msg[STD_MSG_EST_Y], std_msg[STD_MSG_EST_THETA]);
						#endif

						BCI_unlockSem(std_msgSem, "readBuffer")
					}
					break;

				case MPC_MSG_TYPE:
					//Read in mpc msg
					BCI_lockSem(mpc_msgSem, "readBuffer")
					{
						#ifdef UARTHANDLER_DEBUG
							writeDebugStream("MPC Msg: ");
						#endif
						uart_readMsg(mpc_msg, MPC_MSG_LENGTH);

						int j;
						for (int i = 0; i < 3; i++)
						{
							j = i * 9;
							conv.b[0] = mpc_msg[MPC_MSG_X_COORD + j];
							conv.b[1] = mpc_msg[MPC_MSG_X_COORD + 1 + j];
							conv.b[2] = mpc_msg[MPC_MSG_X_COORD + 2 + j];
							conv.b[3] = mpc_msg[MPC_MSG_X_COORD + 3 + j];
							mpc_msg[MPC_MSG_X_COORD + j] = conv.l;

							conv.b[0] = mpc_msg[MPC_MSG_Y_COORD + j];
							conv.b[1] = mpc_msg[MPC_MSG_Y_COORD + 1 + j];
							conv.b[2] = mpc_msg[MPC_MSG_Y_COORD + 2 + j];
							conv.b[3] = mpc_msg[MPC_MSG_Y_COORD + 3 + j];
							mpc_msg[MPC_MSG_Y_COORD + j] = conv.l;

							mpcMsgFlag = true;
							//#ifdef UARTHANDLER_DEBUG
								writeDebugStreamLine("mpc got (%d,%d)", mpc_msg[MPC_MSG_X_COORD + j], mpc_msg[MPC_MSG_Y_COORD + j]);
							//#endif
						}

						BCI_unlockSem(mpc_msgSem, "readBuffer")
					}
					break;

				default:
					break;
			}
		}

		BCI_lockSem(std_msgSem, "readBuffer")
		{
			if (timer_Repeat(&lidarTimer, 100) && std_msg[STD_MSG_LIDAR_RPM] != 0)
			{
				if (std_msg[STD_MSG_LIDAR_RPM] > 125)
					motor[lidar] = motor[lidar] - 1;
				else if (std_msg[STD_MSG_LIDAR_RPM] < 125)
					motor[lidar] = motor[lidar] + 1;
			}

			if (motor[lidar] > 70)
				motor[lidar] = 70;
			else if (motor[lidar] < 30)
				motor[lidar] = 30;

			BCI_unlockSem(std_msgSem, "readBuffer")
		}

		wait1Msec(15);
	}
}

#endif //UARTHANDLER_H_INCLUDED
