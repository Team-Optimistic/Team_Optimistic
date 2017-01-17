#ifndef COLLISIONHANDLER_C_INCLUDED
#define COLLISIONHANDLER_C_INCLUDED

typedef struct statePack_t
{
  long x, y, theta;
} statePack;

long doesDriveCollide(const int mm)
{
statePack c_state;
c_state.x = x;
c_state.y = y;
c_state.theta = theta;
return doesDriveCollideSP(c_state , mm);
}

long doesDriveCollideSP(const statePack &sp, const int mm)  //potential problem: return 0 if successful, what if the distance you can go is also 0?
{

    // Given our current state (x, y, theta, intake, lift),
    // If we drive straight for the given millimeters,
    // Do we hit the field wall?
    // Do we hit the fence?
    // If we do hit something, where along our path do we hit it?
  long s_dist;
  int goal_x = sp.x + (mm * sin(sp.theta));
  int goal_y = sp.y + (mm * cos(sp.theta));
  int x_min, y_min, x_max, y_max, claw_extrem1_x, claw_extrem1_y, claw_extrem2_x, claw_extrem2_y;
  int x_dist = 0; //therefore return 0 if path was successful
  int y_dist = 0;

    x_min = y_min = 10;
    x_max = 3640;
    y_max = 1810;
    if(INTAKE_OPEN){
      claw_extrem1_x = goal_x + (534 * cos(sp.theta));
      claw_extrem2_x = goal_x - (534 * cos(sp.theta));
      claw_extrem1_y = goal_y + (432 * sin(sp.theta));
      claw_extrem2_y = goal_y - (432 * sin(sp.theta));

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
  }
  if(INTAKE_CLOSED){
claw_extrem1_y = claw_extrem2_y = goal_y + (915 * cos(sp.theta));
claw_extrem1_x = claw_extrem2_x = goal_x + (915 * cos(sp.theta));


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
      s_dist = sqrt ( x_dist * x_dist + y_dist * y_dist);

  return s_dist;
}

long doesTurnCollide(const int deg)
{
  statePack c_state;
  c_state.x = x;
  c_state.y = y;
  c_state.theta = theta;
  return doesTurnCollideSP(c_state,deg);
  // Given our current state (x, y, theta, intake, lift),
  // If we turn for the given degrees (clockwise),
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where in the turn do we hit it?
}

long doesTurnCollideSP(const statePack *sp, const int deg)
{
if(LIFT_UP){
//return  doesDriveCollideSP(sp,0); //no longer there, but do need to consider smaller turning circle here
}
else{
long
  return s_deg;
}

}

long doesIntakeCollide(const int mm)
{
  // Given our current state (x, y, theta, intake, lift),
  // If we drive straight for the given millimeters and then close the intake,
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where along our path do we hit it?
}

long doesIntakeCollideSP(const statePack *sp, const int mm)
{
}

#endif //COLLISIONHANDLER_C_INCLUDED
