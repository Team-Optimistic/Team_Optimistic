#ifndef COLLISIONHANDLER_C_INCLUDED
#define COLLISIONHANDLER_C_INCLUDED

long doesDriveCollideSP(const statePack *sp, const int mm)  //potential problem: return 0 if successful, what if the distance you can go is also 0?
{
  // Given our current state (x, y, theta, intake, lift),
  // If we drive straight for the given millimeters,
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where along our path do we hit it?

  long s_dist;
  int goal_x = sp->x + (mm * sin(sp->theta));
  int goal_y = sp->y + (mm * cos(sp->theta));
  int x_min, y_min, x_max, y_max, claw_extrem1_x, claw_extrem1_y, claw_extrem2_x, claw_extrem2_y;
  int x_dist = 0; //therefore return 0 if path was successful
  int y_dist = 0;

  x_min = y_min = 10;
  x_max = 3640;
  y_max = 1810;

  switch (intakeAndLiftTask_intakeState)
  {
    case INTAKE_OPEN:
      claw_extrem1_x = goal_x + (534 * cos(sp->theta));
      claw_extrem2_x = goal_x - (534 * cos(sp->theta));
      claw_extrem1_y = goal_y + (432 * sin(sp->theta));
      claw_extrem2_y = goal_y - (432 * sin(sp->theta));

      if(claw_extrem2_x < x_min){
        x_dist = x_min - claw_extrem2_x;
      }
      if(claw_extrem2_y < y_min){
        y_dist = y_min - claw_extrem2_y;
      }
      if(claw_extrem2_x > x_max){
        x_dist = claw_extrem2_x - x_max;
      }
      if(claw_extrem2_y > y_max){
        y_dist = claw_extrem2_y - y_max;
      }
      break;

    case INTAKE_CLOSED:
      claw_extrem1_y = claw_extrem2_y = goal_y + (915 * cos(sp->theta));
      claw_extrem1_x = claw_extrem2_x = goal_x + (915 * cos(sp->theta));
  }

  if(claw_extrem1_x < x_min ){
    x_dist = x_min - claw_extrem1_x;
  }
  if(claw_extrem1_y < y_min){
    y_dist = y_min - claw_extrem1_y;
  }
  if(claw_extrem1_x > x_max){
    x_dist = claw_extrem1_x - x_max;
  }
  if(claw_extrem1_y > y_max){
    y_dist = claw_extrem1_y - y_max;
  }
  s_dist = sqrt (x_dist * x_dist + y_dist * y_dist);

  return s_dist;
}

long doesDriveCollide(const int mm)
{
  statePack c_state;
  BCI_lockSem(std_msgSem, "doesDriveCollide")
  {
    c_state.x = std_msg[STD_MSG_EST_X];
    c_state.y = std_msg[STD_MSG_EST_Y];
    c_state.theta = std_msg[STD_MSG_EST_THETA];
    BCI_unlockSem(std_msgSem, "doesDriveCollide")
  }
  return doesDriveCollideSP(&c_state , mm);
}

long doesTurnCollideSP(const statePack *sp, const int deg)
{
	switch (intakeAndLiftTask_liftState)
	{
		case LIFT_UP:
     //return  doesDriveCollideSP(sp,0); //no longer there, but do need to consider smaller turning circle here
		 break;

		case LIFT_DOWN:
			break;
	}
}

long doesTurnCollide(const int deg)
{
  // Given our current state (x, y, theta, intake, lift),
  // If we turn for the given degrees (clockwise),
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where in the turn do we hit it?

  statePack c_state;
  BCI_lockSem(std_msgSem, "doesTurnCollide")
  {
    c_state.x = std_msg[STD_MSG_EST_X];
    c_state.y = std_msg[STD_MSG_EST_Y];
    c_state.theta = std_msg[STD_MSG_EST_THETA];
    BCI_unlockSem(std_msgSem, "doesTurnCollide")
  }
  return doesTurnCollideSP(&c_state, deg);
}

long doesIntakeCollideSP(const statePack *sp, const int mm)
{
}

long doesIntakeCollide(const int mm)
{
  // Given our current state (x, y, theta, intake, lift),
  // If we drive straight for the given millimeters and then close the intake,
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where along our path do we hit it?
}

#endif //COLLISIONHANDLER_C_INCLUDED
