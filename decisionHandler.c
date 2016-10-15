#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED

#include "uartHandler.c"

static short iso_x = -1, iso_y = -1;

/*
Returns whether the input coordinates are the same as the last
@param x New x coord
@param y New y coord
@return Whether the input coordinates are the same as the last
 */
bool isSameObject(const short x, const short y)
{
  //Maximum deviance to be considered a new object
  const float maxDeviance = 0.1;

  //First call
  if (iso_x == -1 || iso_y == -1)
  {
    return false;
  }

  const bool out = (fabs(x - iso_x) < maxDeviance) ||
                   (fabs(y - iso_y) < maxDeviance);
  iso_x = x;
  iso_y = y;

  return out;

}

//Drives the robot based on recieved commands
task commandRobot()
{
  short xDemand, yDemand;

  while (true)
  {
    if (!isWorking)
    {
      semaphoreLock(msgSem);

      if (bDoesTaskOwnSemaphore(msgSem))
      {
        xDemand = msg[MSG_X_COORD];
        yDemand = msg[MSG_Y_COORD];

        if (bDoesTaskOwnSemaphore(msgSem))
        {
          semaphoreUnlock(msgSem);
        }

        switch (msg[MSG_PICKUP])
        {
          case MSG_PICKUP_NONE:
            if (!isSameObject(xDemand, yDemand))
            {
              moveToPoint(xDemand, yDemand);
            }
            break;

          case MSG_PICKUP_STAR:
            if (!isSameObject(xDemand, yDemand))
            {
              pickUpStar(xDemand, yDemand);
            }
            break;

          case MSG_PICKUP_CUBE:
            if (!isSameObject(xDemand, yDemand))
            {
              pickUpCube(xDemand, yDemand);
            }
            break;

          default:
            break;
        }
      }
    }
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
