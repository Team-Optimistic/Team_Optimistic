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
	SPC msg send structure is
	<header>
	<short message id>

	SPC msg recieve structure is
	<header>
	<short x coordinate demand>
	<short y coordinate demand>
	<short pick up object> // 0 = no object
												 // 1 = star
												 // 2 = cube
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
 	<short x coordinate demand>
 	<short y coordinate demand>
 	<short pick up object> // 0 = do not pick up
 												 // 1 = pick up star
 												 // 2 = pick up cube
 */

//Standard message
#define STD_MSG_LENGTH 10
long std_msg[STD_MSG_LENGTH];
#define STD_MSG_EST_X     0
#define STD_MSG_EST_Y     4
#define STD_MSG_EST_THETA 8
#define STD_MSG_LIDAR_RPM 9

//Message to get objects behind us
#define SPC_MSG_LENGTH 3
long spc_msg[SPC_MSG_LENGTH];
#define SPC_MSG_X_COORD 0
#define SPC_MSG_Y_COORD 1
#define SPC_MSG_PICKUP  2
#define SPC_MSG_PICKUP_CLEAR 0
#define SPC_MSG_PICKUP_STAR  1
#define SPC_MSG_PICKUP_CUBE  2

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

//Message write semaphore, always get lock before reading
//Only task readBuffer() may write to msg
TSemaphore std_msgSem, spc_msgSem, mpc_msgSem;

//General UART semaphore, always get lock before writing
//Only task readBuffer() may read from UART
TSemaphore uartSem;

//Message count
#define MSG_COUNT_LENGTH 4
short msgCount[MSG_COUNT_LENGTH];
#define MSG_COUNT_TOTAL 0
#define MSG_COUNT_STD   1
#define MSG_COUNT_SPC   2
#define MSG_COUNT_MPC   3

//Message types
#define STD_MSG_TYPE 1
#define SPC_MSG_TYPE 2
#define MPC_MSG_TYPE 3

/*
Initializes everything this file needs for comms with the pi
 */
void initUART()
{
	//Initialize counts
	for (unsigned int i = 0; i < MSG_COUNT_LENGTH; i++)
	{
		msgCount[i] = 0;
	}

	std_msg[STD_MSG_EST_X] = 0;
	std_msg[STD_MSG_EST_Y] = 0;
	std_msg[STD_MSG_EST_THETA] = 0;
	std_msg[STD_MSG_LIDAR_RPM] = 250;

	//Initialize semaphores
	semaphoreInitialize(std_msgSem);
	semaphoreInitialize(spc_msgSem);
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
	if (type > 3)
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
	if (type > 3)
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
		stdMsgUnion.l = SensorValue[leftQuad];
		sendChar(UART1, stdMsgUnion.b[0]);
		sendChar(UART1, stdMsgUnion.b[1]);
		sendChar(UART1, stdMsgUnion.b[2]);
		sendChar(UART1, stdMsgUnion.b[3]);

		stdMsgUnion.l = SensorValue[rightQuad];
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
Requests the pi send back the position of an object behind the robot
 */
void sendSPCMsg()
{
	BCI_lockSem(uartSem, "sendSPCMsg")
	{
		//Send header
		uart_sendMessageHeader(SPC_MSG_TYPE);

		//Reset MPC flag
		mpcMsgFlag = false;

		while (!bXmitComplete(UART1)) { wait1Msec(1); }

		#ifdef UARTHANDLER_DEBUG
			writeDebugStreamLine("UART Handler: Sent SPC msg");
		#endif

		BCI_unlockSem(uartSem, "sendSPCMsg")
	}
}

/*
Tells the pi what object we just picked up
 */
void sendMPCMsg()
{
	BCI_lockSem(uartSem, "sendMPCMsg")
	{
		//Send start byte
		sendChar(UART1, 0xFA);

		//Send msg type
		sendChar(UART1, MPC_MSG_TYPE);

		//Send msg count
		sendChar(UART1, msgCount[MSG_COUNT_MPC]);

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

	long2Bytes conv;

	while (true)
	{
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

						#ifdef UARTHANDLER_DEBUG_READ
							writeDebugStreamLine("decoded: %d, %d", std_msg[STD_MSG_EST_X], std_msg[STD_MSG_EST_Y]);
						#endif

						BCI_unlockSem(std_msgSem, "readBuffer")
					}
					break;

				case SPC_MSG_TYPE:
					//Read in spc msg
					BCI_lockSem(spc_msgSem, "readBuffer")
					{
						#ifdef UARTHANDLER_DEBUG
							writeDebugStream("SPC Msg: ");
						#endif
						uart_readMsg(spc_msg, SPC_MSG_LENGTH);

						BCI_unlockSem(spc_msgSem, "readBuffer")
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

						mpcMsgFlag = true;

						for (int i = 0; i < 4; i++)
						{
							conv.b[0] = mpc_msg[MPC_MSG_X_COORD + (i * 9)];
							conv.b[1] = mpc_msg[MPC_MSG_X_COORD + 1 + (i * 9)];
							conv.b[2] = mpc_msg[MPC_MSG_X_COORD + 2 + (i * 9)];
							conv.b[3] = mpc_msg[MPC_MSG_X_COORD + 3 + (i * 9)];
							mpc_msg[MPC_MSG_X_COORD + (i * 9)] = conv.l;

							conv.b[0] = mpc_msg[MPC_MSG_Y_COORD + (i * 9)];
							conv.b[1] = mpc_msg[MPC_MSG_Y_COORD + 1 + (i * 9)];
							conv.b[2] = mpc_msg[MPC_MSG_Y_COORD + 2 + (i * 9)];
							conv.b[3] = mpc_msg[MPC_MSG_Y_COORD + 3 + (i * 9)];
							mpc_msg[MPC_MSG_Y_COORD + (i * 9)] = conv.l;

							#ifdef UARTHANDLER_DEBUG
								writeDebugStreamLine("mpc got (%d,%d)", mpc_msg[MPC_MSG_X_COORD + (i * 9)], mpc_msg[MPC_MSG_Y_COORD + (i * 9)]);
							#endif
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
			if (std_msg[STD_MSG_LIDAR_RPM] != 0)
			{
				if (std_msg[STD_MSG_LIDAR_RPM] > 125)
				{
					motor[lidar] = motor[lidar] - 1;
				}
				else if (std_msg[STD_MSG_LIDAR_RPM] < 125)
				{
					motor[lidar] = motor[lidar] + 1;
				}
			}

			BCI_unlockSem(std_msgSem, "readBuffer")
		}

		wait1Msec(15);
	}
}

#endif //UARTHANDLER_H_INCLUDED
