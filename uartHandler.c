#ifndef UARTHANDLER_H_INCLUDED
#define UARTHANDLER_H_INCLUDED

/*
	Message header is
	<start byte 0xFA>
	<msg type byte>
	<short msg count>

	Message send structure is
	<header>
	<short intake pot val>
	<short left quad val>
	<short right quad val>

	Message recieve structure is
	<header>
	<short estimated x>
	<short estimated y>
	<short estimated theta>
	<short x coordinate demand>
	<short y coordinate demand>
	<short pick up object> // 0 = do not pick up
												 // 1 = pick up star
												 // 2 = pick up cube
*/

/*
	Special message send structure is
	<header>
	<short message id>

	Special message recieve structure is
	<header>
	<short x coordinate demand>
	<short y coordinate demand>
	<short pick up object> // 0 = no object
												 // 1 = star
												 // 2 = cube
 */

//Standard message
#define STD_MSG_LENGTH 7
short std_msg[STD_MSG_LENGTH];
#define STD_MSG_COUNT     0
#define STD_MSG_EST_X     1
#define STD_MSG_EST_Y     2
#define STD_MSG_EST_THETA 3
#define STD_MSG_X_COORD   4
#define STD_MSG_Y_COORD   5
#define STD_MSG_PICKUP    6
#define STD_MSG_PICKUP_NONE 0
#define STD_MSG_PICKUP_STAR 1
#define STD_MSG_PICKUP_CUBE 2

//Message to get objects behind us
#define SPC_MSG_LENGTH 3
short spc_msg[SPC_MSG_LENGTH];
#define SPC_MSG_X_COORD 0
#define SPC_MSG_Y_COORD 1
#define SPC_MSG_PICKUP  2
#define SPC_MSG_PICKUP_CLEAR 0
#define SPC_MSG_PICKUP_STAR  1
#define SPC_MSG_PICKUP_CUBE  2

//Message write semaphore, always get lock before reading
//Only task readBuffer() may write to msg
TSemaphore std_msgSem;

//Special message semaphore, always get lock before reading
//Only task readBuffer() may write to spc_msg
TSemaphore spc_msgSem;

//General UART semaphore, always get lock before writing
//Only task readBuffer() may read from UART
TSemaphore uartSem;

//Message count
#define MSG_COUNT_LENGTH 3
short msgCount[MSG_COUNT_LENGTH];
#define MSG_COUNT_TOTAL 0
#define MSG_COUNT_STD   1
#define MSG_COUNT_SPC   2

//Message types
#define STD_MSG_TYPE MSG_COUNT_STD
#define SPC_MSG_TYPE MSG_COUNT_SPC

/*
Initializes everything this file needs for comms with the pi
 */
#warning "initUART"
void initUART()
{
	for (unsigned int i = 0; i < MSG_COUNT_LENGTH; i++)
	{
		msgCount[i] = 0;
	}

	semaphoreInitialize(std_msgSem);
	semaphoreInitialize(spc_msgSem);
	semaphoreInitialize(uartSem);

	setBaudRate(UART1, baudRate9600);
}

/*
Generates new message count without side-effects
@param type Message type
 */
short uart_getMessageCount_Soft(const short type)
{
	return msgCount[type] + 1 >= 0xFF ? 0 : msgCount[type] + 1;
}

/*
Generates new message count
@param type Message type
 */
short uart_getMessageCount(const short type)
{
	return msgCount[type] + 1 >= 0xFF ? 0 : (msgCount[type] = msgCount[type] + 1);
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
}

/*
Sends a standard message to the pi
 */
#warning "sendStandardMessage"
void sendStandardMessage()
{
	BCI_lockSem(uartSem, "sendCurrentData")
	{
		//Send header
		uart_sendMessageHeader(STD_MSG_TYPE);

		//Send analog data
		sendChar(UART1, SensorValue[intakePot]);

		//Send digital data
		sendChar(UART1, SensorValue[leftQuad]);
		sendChar(UART1, SensorValue[rightQuad]);

		BCI_unlockSem(uartSem, "sendCurrentData")
	}
}

/*
Requests the pi send back the position of an object behind the robot
 */
#warning "sendGetBehindRequest"
void sendGetBehindRequest()
{
	BCI_lockSem(uartSem, "sendGetBehindRequest")
	{
		//Send header
		uart_sendMessageHeader(SPC_MSG_TYPE);

		BCI_unlockSem(uartSem, "sendGetBehindRequest")
	}
}

/*
Reads in a message
 */
void uart_readMsg(short *msg, const int length)
{
	for (unsigned int index = 0; index < length; index++)
	{
		//Trash 0xFF
		if ((msg[index] = getChar(UART1)) == 0xFF)
		{
			index--;
		}
	}
}

/*
Polls uart for a message and records it into msg[]
 */
#warning "readBuffer"
task readBuffer()
{
	short msgFlagHolder, msgTypeFlagHolder, msgCountFlagHolder;

	while (true)
	{
		//Load start byte into temp variable
		if ((msgFlagHolder = getChar(UART1)) == 0xFA)
		{
			BCI_lockSem(std_msgSem, "readBuffer")
			{
				//Get msg type
				while ((msgTypeFlagHolder = getChar(UART1)) == 0xFF) { wait1Msec(1); }

				//Get msg count
				while ((msgCountFlagHolder = getChar(UART1)) == 0xFF) { wait1Msec(1); }

				//Verify msg count
				if (!uart_verifyMessageCount(msgTypeFlagHolder, msgCountFlagHolder))
				{
					writeDebugStreamLine("Bad msg count");
				}

				switch (msgTypeFlagHolder)
				{
					case STD_MSG_TYPE:
						//Read in std msg
						uart_readMsg(std_msg, STD_MSG_LENGTH);
						break;

					case SPC_MSG_TYPE:
						//Read in spc msg
						uart_readMsg(spc_msg, SPC_MSG_LENGTH);
						break;

					default:
						break;
				}

				BCI_unlockSem(std_msgSem, "readBuffer")
			}
		}

		//Task wait
		wait1Msec(1);
	}
}

#endif //UARTHANDLER_H_INCLUDED
