#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED

//Drives the robot based on recieved commands
task commandRobot()
{
  long xDemand[MPC_MSG_OBJ_COUNT], yDemand[MPC_MSG_OBJ_COUNT], pickup[MPC_MSG_OBJ_COUNT];
  bool badData = false;
	sendMPCMsg();
	wait1Msec(100);
  while (true)
  {
    if (mpcMsgFlag && !badData)
    {
    	writeDebugStreamLine("MPC : Flag");
      BCI_lockSem(mpc_msgSem, "commandRobot")
      {
        int j;
        for (int i = 0; i < MPC_MSG_OBJ_COUNT; i++)
        {
          j = i * 9;
          xDemand[i] = mpc_msg[MPC_MSG_X_COORD + j];
          yDemand[i] = mpc_msg[MPC_MSG_Y_COORD + j];
          pickup[i] = mpc_msg[MPC_MSG_PICKUP + j];
          writeDebugStreamLine("%d,%d,%d", xDemand[i], yDemand[i], pickup[i]);
        }

        BCI_unlockSem(mpc_msgSem, "commandRobot")
      }

      //Check for bad data
      for (int i = 0; i < MPC_MSG_OBJ_COUNT; i++)
      {
        if (xDemand[i] < -1 ||
            xDemand[i] > 3658 ||
            yDemand[i] < -1 ||
            yDemand[i] > 1829)
        {
          badData = true;
          writeDebugStreamLine("MPC: BAD DATA");
          break;
        }
        else{
        	badData=false;
        }
      }

      //Break out if we get bad data from pi
      if (badData)
      {
        //break;
      }

      //Go through each msg and perform the action
      for (int i = 0; i < MPC_MSG_OBJ_COUNT; i++)
      {
        switch (pickup[i])
        {
          case MPC_MSG_PICKUP_CLEAR:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("moving to point (%d,%d)", xDemand[i], yDemand[i]);
            #endif

            moveToPoint(xDemand[i], yDemand[i]);
            break;

          case MPC_MSG_PICKUP_STAR:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("getting star at (%d,%d)", xDemand[i], yDemand[i]);
            #endif

            if( pickUp(xDemand[i], yDemand[i] , false))
            	i = MPC_MSG_OBJ_COUNT;//totally done
            break;

          case MPC_MSG_PICKUP_CUBE:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("getting cube at (%d,%d)", xDemand[i], yDemand[i]);
            #endif

            pickUp(xDemand[i], yDemand[i], true);
            i = MPC_MSG_OBJ_COUNT;//totally done
            break;

          case MPC_MSG_PICKUP_BACK:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("moving to point (%d,%d) backwards", xDemand[i], yDemand[i]);
            #endif

            moveToPoint(xDemand[i], yDemand[i], true);
            break;

          case MPC_MSG_PICKUP_WALL:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("knocking stars off fence %d", xDemand[i]);
            #endif

            switch (xDemand[i])
            {
              case 1:
              scoreFence(FENCE_LEFT);
              break;

              case 2:
              scoreFence(FENCE_MIDDLE);
              break;

              case 3:
              scoreFence(FENCE_RIGHT);
              break;

              default:
              break;
            }
            break;

          default:
            break;
        }
      }

      //Score whats in the intake
      writeDebugStreamLine("closing claw");
      intakeAndLiftTask_intakeState = INTAKE_CLOSED;
      intakeAndLiftTask_liftState = LIFT_DOWN;
      wait1Msec(650);
      driveStraight_Ballsy(-400);
      intakeAndLiftTask_liftState = LIFT_HALF;
      wait1Msec(250);
      writeDebugStreamLine("about to dump");
      dumpIntake();
      writeDebugStreamLine("dumped");

      mpcMsgFlag = false;
    }

    sendMPCMsg();
    wait1Msec(200);
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
