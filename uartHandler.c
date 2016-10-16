#ifndef UARTHANDLER_H_INCLUDED
#define UARTHANDLER_H_INCLUDED

/*
	Message send structure is
	<start byte 0xFA>
	<short message count>
	<short intake pot val>
	<short left quad val>
	<short right quad val>

	Message recieve structure is
	<start byte 0xFA>
	<short message count>
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
	<start byte 0xFA>
	<start byte 0xFA>
	<short message count>
	<short message id>

	Special message recieve structure is
	<start byte 0xFA>
	<start byte 0xFA>
	<short x coordinate demand>
	<short y coordinate demand>
	<short pick up object> // 0 = no object
												 // 1 = star
												 // 2 = cube
 */

//Message length (excluding start byte)
#define MSG_LENGTH 7

//Current message
short msg[MSG_LENGTH];
#define MSG_COUNT     0
#define MSG_EST_X     1
#define MSG_EST_Y     2
#define MSG_EST_THETA 3
#define MSG_X_COORD   4
#define MSG_Y_COORD   5
#define MSG_PICKUP    6

//Pickup types
#define MSG_PICKUP_NONE 0
#define MSG_PICKUP_STAR 1
#define MSG_PICKUP_CUBE 2

//Special message for backing up
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
TSemaphore msgSem;

//Special message semaphore, always get lock before reading
//Only task readBuffer() may write to spc_msg
TSemaphore spc_msgSem;

//General UART semaphore, always get lock before writing
//Only task readBuffer() may read from UART
TSemaphore uartSem;

//Total message count
static short msgCount;

/*
Initializes everything this file needs for comms with the pi
 */
#warning "initUART"
void initUART()
{
	msgCount = 0;
	semaphoreInitialize(msgSem);
	semaphoreInitialize(spc_msgSem);
	semaphoreInitialize(uartSem);
	setBaudRate(UART1, baudRate9600);
}

/*
Sends a standard message to the pi
 */
#warning "sendCurrentData"
void sendCurrentData()
{
	BCI_lockSem(uartSem, "sendCurrentData")
	{
		//Send start byte
		sendChar(UART1, 0xFA);

		//Send msg header
		sendChar(UART1, msgCount++);

		//Send analog data
		sendChar(UART1, (short)SensorValue[intakePot]);

		//Send digital data
		sendChar(UART1, (short)SensorValue[leftQuad]);
		sendChar(UART1, (short)SensorValue[rightQuad]);

		BCI_unlockSem(uartSem, "sendCurrentData")
	}
}

/*
Requests the pi send back the position of an object behind the robot
 */
#warning "sendGetBehindRequest"
void sendGetBehindRequest(const short msg_id)
{
	BCI_lockSem(uartSem, "sendGetBehindRequest")
	{
		//Send double start byte
		sendChar(UART1, 0xFA);
		sendChar(UART1, 0xFA);

		//Send msg header
		sendChar(UART1, msgCount++);

		//Send msg id
		sendCHar(UART1, msg_id);

		BCI_unlockSem(uartSem, "sendGetBehindRequest")
	}
}

/*
Polls uart for a message and records it into msg[]
 */
#warning "readBuffer"
task readBuffer()
{
	unsigned int index = 0;
	short msgFlagHolder, msgDoubleFlagHolder;

	while (true)
	{
		//Load start byte into temp variable
		if ((msgFlagHolder = getChar(UART1)) == 0xFA)
		{
			BCI_lockSem(msgSem, "readBuffer")
			{
				//If start flag is seen, read in rest of message
				for (index = 0; index < MSG_LENGTH; index++)
				{
					//If double flag, break and treat specially
					if (index == 0 && (msgDoubleFlagHolder = getChar(UART1)) == 0xFA)
					{
						//Read in special message
						for (index = 0; index < SPC_MSG_LENGTH; index++)
						{
							if ((spc_msg[index] = getChar(UART1)) == 0xFF)
							{
								index--;
							}
						}
						break;
					}
					//0xFF represents empty, so ignore it
					else if (msgDoubleFlagHolder == 0xFF)
					{
						msgDoubleFlagHolder = 0;
						index--;
					}
					//msgDoubleFlagHolder has our data, take it
					else if (msgDoubleFlagHolder != 0xFF)
					{
						msgDoubleFlagHolder = 0;
						msg[index] = msgDoubleFlagHolder;
					}
					//0xFF represents empty, so ignore it
					else if (msg[index] = getChar(UART1)) == 0xFF)
					{
						index--;
					}
				}

				BCI_unlockSem(msgSem)
			}
		}

		//Task wait
		wait1Msec(1);
	}
}

#endif //UARTHANDLER_H_INCLUDED
