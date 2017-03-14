#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED

//Drives the robot based on recieved commands
task commandRobot()
{
  long xDemand[MPC_MSG_OBJ_COUNT], yDemand[MPC_MSG_OBJ_COUNT], pickup[MPC_MSG_OBJ_COUNT];

  while (true)
  {
    if (mpcMsgFlag)
    {
      BCI_lockSem(mpc_msgSem, "commandRobot")
      {
        int j;
        for (int i = 0; i < MPC_MSG_OBJ_COUNT; i++)
        {
          j = i * 5;
          xDemand[i] = mpc_msg[MPC_MSG_X_COORD + j];
          yDemand[i] = mpc_msg[MPC_MSG_Y_COORD + j];
          pickup[i] = mpc_msg[MPC_MSG_PICKUP + j];
        }

        BCI_unlockSem(mpc_msgSem, "commandRobot")
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
            sendMPCMsg();
            break;

          case MPC_MSG_PICKUP_STAR:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("getting star at (%d,%d)", xDemand[i], yDemand[i]);
            #endif

            pickUpStar(xDemand[i], yDemand[i]);
            sendMPCMsg();
            break;

          case MPC_MSG_PICKUP_CUBE:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("getting cube at (%d,%d)", xDemand[i], yDemand[i]);
            #endif

            pickUpCube(xDemand[i], yDemand[i]);
            sendMPCMsg();
            break;

          case MPC_MSG_PICKUP_BACK:
            #ifdef MPC_DEBUG
              writeDebugStreamLine("moving to point (%d,%d) backwards", xDemand[i], yDemand[i]);
            #endif

            moveToPoint(xDemand[i], yDemand[i], true);
            sendMPCMsg();
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

            sendMPCMsg();
            break;

          default:
            break;
        }
      }

      //Score whats in the intake
      intakeAndLiftTask_intakeState = INTAKE_CLOSED;
      intakeAndLiftTask_liftState = LIFT_DOWN;
      wait1Msec(500);
      dumpIntake();

      mpcMsgFlag = false;
    }

    wait1Msec(15);
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
