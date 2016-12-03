#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED


//Drives the robot based on recieved commands
task commandRobot()
{
  long xDemand[3], yDemand[3], pickup[3];

  while (true)
  {
  	if (mpcMsgFlag)
  	{
	    BCI_lockSem(mpc_msgSem, "commandRobot")
	    {
	      for (int i = 0; i < 3; i++)
	      {
	        xDemand[i] = mpc_msg[MPC_MSG_X_COORD + (i * 5)];
	        yDemand[i] = mpc_msg[MPC_MSG_Y_COORD + (i * 5)];
	        pickup[i] = mpc_msg[MPC_MSG_PICKUP];
	      }
	      BCI_unlockSem(mpc_msgSem, "commandRobot")
	    }

      //First instruction determines type of following ones
      //We can only follow multiple instructions if we are getting stars
      switch (pickup[0])
      {
        case MPC_MSG_PICKUP_CLEAR:
        	writeDebugStreamLine("moving to point (%d,%d)", xDemand[0], yDemand[0]);

        	SensorValue[LED_Waiting] = LED_OFF;
        	SensorValue[LED_Driving] = LED_ON;

          moveToPoint(xDemand[0], yDemand[0]);

          SensorValue[LED_Driving] = LED_OFF;

          sendMPCMsg();
          break;

        case MPC_MSG_PICKUP_STAR:
					writeDebugStreamLine("getting star at (%d,%d)", xDemand[0], yDemand[0]);

        	SensorValue[LED_Waiting] = LED_OFF;
					SensorValue[LED_Stars] = LED_ON;

          pickUpStars(xDemand, yDemand);

					SensorValue[LED_Stars] = LED_OFF;

          sendMPCMsg();
          break;

        case MPC_MSG_PICKUP_CUBE:
					writeDebugStreamLine("getting cube at (%d,%d)", xDemand[0], yDemand[0]);

        	SensorValue[LED_Waiting] = LED_OFF;
					SensorValue[LED_Cube] = LED_ON;

          pickUpCube(xDemand[0], yDemand[0]);

					SensorValue[LED_Cube] = LED_OFF;

          sendMPCMsg();
          break;

				case MPC_MSG_PICKUP_BACK:
					writeDebugStreamLine("moving to point (%d,%d) backwards", xDemand[0], yDemand[0]);

        	SensorValue[LED_Waiting] = LED_OFF;
					SensorValue[LED_Driving] = LED_ON;

					moveToPoint(xDemand[0], yDemand[0], 0, true);

					SensorValue[LED_Driving] = LED_OFF;

					sendMPCMsg();
					break;

				case MPC_MSG_PICKUP_WALL:
					writeDebugStreamLine("knocking stars off fence %d", xDemand[0]);

        	SensorValue[LED_Waiting] = LED_OFF;
					SensorValue[LED_Fence] = LED_ON;

					switch (xDemand[0])
					{
						case 1:
							scoreFence(FENCE_LEFT);
							break;

						case 2:
							scoreFence(FENCE_MIDDLE);
							break;

						default:
							scoreFence(FENCE_RIGHT);
							break;
					}

					SensorValue[LED_Fence] = LED_OFF;

					sendMPCMsg();
					break;

        default:
        	SensorValue[LED_Waiting] = LED_ON;
          break;
      }

      mpcMsgFlag = false;
	  }

    wait1Msec(5);
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
