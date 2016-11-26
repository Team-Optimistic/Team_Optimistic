	#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED

static long iso_x = -1, iso_y = -1;

/*
Returns whether the input coordinates are the same as the last
@param x New x coord
@param y New y coord
@return Whether the input coordinates are the same as the last
 */
bool isSameObject(const long x, const long y)
{
  //Maximum deviance to be considered a new object
  const float maxDeviance = 0.1;

  //First call
  if (iso_x == -1 || iso_y == -1)
  {
    return false;
  }

  bool out = (fabs(x - iso_x) < maxDeviance) ||
             (fabs(y - iso_y) < maxDeviance);

  iso_x = x;
  iso_y = y;

  return out;

}

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
          if (!isSameObject(xDemand[0], yDemand[0]))
          {
            moveToPoint(xDemand[0], yDemand[0]);
            sendMPCMsg();
          }
          break;

        case MPC_MSG_PICKUP_STAR:
					writeDebugStreamLine("getting star at (%d,%d)", xDemand[0], yDemand[0]);
          pickUpStars(xDemand, yDemand);
          sendMPCMsg();
          break;

        case MPC_MSG_PICKUP_CUBE:
					writeDebugStreamLine("getting cube at (%d,%d)", xDemand[0], yDemand[0]);
          if (!isSameObject(xDemand[0], yDemand[0]))
          {
            pickUpCube(xDemand[0], yDemand[0]);
            sendMPCMsg();
          }
          break;

        default:
          break;
      }

      mpcMsgFlag = false;
	  }

    wait1Msec(5);
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
