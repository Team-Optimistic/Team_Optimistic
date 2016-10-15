#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED

#include "uartHandler.c"

static short iso_x = -1, iso_y = -1;

/*
Returns whether the input coordinates are the same as the last
@param x New x coord
@param y New y coord
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
  while (true)
  {
    if (!isWorking)
    {
      semaphoreLock(msgSem);

      if (bDoesTaskOwnSemaphore(msgSem))
      {
        switch (msg[MSG_PICKUP])
        {
          case MSG_PICKUP_NONE:
            if (!isSameObject(msg[MSG_X_COORD], msg[MSG_Y_COORD]))
            {
              moveToPoint(msg[MSG_X_COORD], msg[MSG_Y_COORD]);
            }
            break;

          case MSG_PICKUP_STAR:
            if (!isSameObject(msg[MSG_X_COORD], msg[MSG_Y_COORD]))
            {
              pickUpStar(msg[MSG_X_COORD], msg[MSG_Y_COORD]);
            }
            break;

          case MSG_PICKUP_CUBE:
            break;

          default:
            break;
        }

        if (bDoesTaskOwnSemaphore(msgSem))
        {
          semaphoreUnlock(msgSem);
        }
      }
    }
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
