#ifndef DECISIONHANDLER_C_INCLUDED
#define DECISIONHANDLER_C_INCLUDED

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
  short xDemand, yDemand, pickup;

  while (true)
  {
    BCI_lockSem(std_msgSem, "commandRobot")
    {
      xDemand = std_msg[STD_MSG_X_COORD];
      yDemand = std_msg[STD_MSG_Y_COORD];
      pickup = std_msg[STD_MSG_PICKUP];
      BCI_unlockSem(std_msgSem, "commandRobot")
    }

      switch (pickup)
      {
        case STD_MSG_PICKUP_NONE:
          if (!isSameObject(xDemand, yDemand))
          {
            moveToPoint(xDemand, yDemand);
          }
          break;

        case STD_MSG_PICKUP_STAR:
          if (!isSameObject(xDemand, yDemand))
          {
            pickUpStar(xDemand, yDemand);
          }
          break;

        case STD_MSG_PICKUP_CUBE:
          if (!isSameObject(xDemand, yDemand))
          {
            pickUpCube(xDemand, yDemand);
          }
          break;

        default:
          break;
      }

    wait1Msec(1);
  }
}

#endif //DECISIONHANDLER_C_INCLUDED
