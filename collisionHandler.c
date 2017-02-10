#ifndef COLLISIONHANDLER_C_INCLUDED
#define COLLISIONHANDLER_C_INCLUDED

/*struct claw_extremity{        //include x, y for claw extremity 1 and 2
int x_val;
int y_val;
} front_right, front_left, back_right, back_left;*/
typedef struct{
  int x_val;
  int y_val;
} claw_extremity;

claw_extremity front_left;
claw_extremity front_right;
claw_extremity back_left;
claw_extremity back_right;
const int dist_cent_to_corner = 432; //432mm
const int dist_cent_to_open_claw_x = 635;// 25 inches from center
const int dist_cent_to_open_claw_y = 321;
const int dist_cent_to_closed_claw = 851;// 33.5 inches
const int ft_to_mm = 26;//dist from center to edge of robot
const int field_right_wall = 3658; //3658mm to right wall = 12ft
const int field_fence = 1829;//fence at y = 1829mm = 6ft
int x_min = ft_to_mm;
int y_min = ft_to_mm;   //edge of robot is against wall
int x_max = field_right_wall - ft_to_mm;
int y_max = field_fence - ft_to_mm;
int f_rad;



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


long doesDriveCollideSP(const statePack *sp, const int mm)  //potential problem: return 0 if successful, what if the distance you can go is also 0?
{
  // Given our current state (x, y, theta, intake, lift),
  // If we drive straight for the given millimeters,
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where along our path do we hit it?
  long s_dist;
  int goal_x = sp->x + (mm * sindegrees(sp->theta));
  int goal_y = sp->y + (mm * cosdegrees(sp->theta));
  int x_dist = 0; //therefore return 0 if path was successful
  int y_dist = 0;

  switch (intakeAndLiftTask_intakeState)
  {
    case INTAKE_OPEN:
      setOpenIntakeVals();
      break;
    case INTAKE_CLOSED:
      setClosedIntakeVals();
      break;
      // case default :
      ////////////////something, for when state is wait or rest
      // break;
  }
    if(front_left.x_val > x_max){
      x_dist = front_left.x_val - x_max;
    }
    if(front_left.y_val > y_max){
      y_dist = front_left.y_val - y_max;
    }
    if(front_right.x_val > x_max){
      x_dist = front_right.x_val - x_max;
    }
    if(front_right.y_val > y_max){
      y_dist = front_right.y_val - y_max;
    }

    s_dist = sqrt (x_dist * x_dist + y_dist * y_dist);
    return s_dist;
}


long doesTurnCollideSP(const statePack *sp; const int deg)
{
  back_left.x_val = sp->x - ft_to_mm;
  back_right.x_val = sp->x + ft_to_mm;
  back_left.y_val = sp->y - ft_to_mm;
  back_right.y_val = sp->y + ft_to_mm;
	switch (intakeAndLiftTask_liftState)
	{
    case LIFT_UP:
      front_right.x_val = sp->x + (ft_to_mm * cosdegrees(sp->theta));
      front_left.x_val = sp->x - (ft_to_mm * cosdegrees(sp->theta));
      front_right.y_val = sp->y + (ft_to_mm * sindegrees(sp->theta));
      front_left.y_val = sp->y - (ft_to_mm * sindegrees(sp->theta));
      f_rad = dist_cent_to_corner;
      break;
    case LIFT_DOWN:
      if(intakeAndLiftTask_intakeState == INTAKE_OPEN){
          setOpenIntakeVals();
      }else if(intakeAndLiftTask_intakeState == INTAKE_CLOSED){
          setClosedIntakeVals();
      }
    break;
}
long turndeg = 0;
  while(testCornerCollision && (turndeg <= deg)){
    front_left.x_val += f_rad * cos(turndeg + sp->theta);
    front_right.x_val += f_rad * cos(turndeg + sp->theta);
    front_left.y_val += f_rad * sin(turndeg + sp->theta);
    front_right.y_val += f_rad * sin(turndeg + sp->theta);
    back_left.x_val += dist_cent_to_corner * cos(turndeg + sp->theta);
    back_right.x_val += dist_cent_to_corner * cos(turndeg + sp->theta);
    back_left.y_val += dist_cent_to_corner * sin(turndeg + sp->theta);
    back_right.y_val += dist_cent_to_corner * sin(turndeg + sp->theta);
    turndeg++;
 }
 return(turndeg);
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

long doesindegreestakeCollideSP(const statePack *sp, const int mm)
{
}

long doesindegreestakeCollide(const int mm)
{
// Given our current state (x, y, theta, intake, lift),
  // If we drive straight for the given millimeters and then close the intake,
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where along our path do we hit it?
  statePack c_state;
  BCI_lockSem(std_msgSem, "doesindegreestakeCollide")
  {
    c_state.x = std_msg[STD_MSG_EST_X];
    c_state.y = std_msg[STD_MSG_EST_Y];
    c_state.theta = std_msg[STD_MSG_EST_THETA];
    BCI_unlockSem(std_msgSem, "doesindegreestakeCollide")
  }
  return doesindegreestakeCollideSP(&c_state, mm);
}

bool testCornerCollision(){
  if((front_left.x_val | front_right.x_val | back_left.x_val | back_right.x_val) > x_max){
    return false;
  }
  else if((front_left.x_val | front_right.x_val | back_left.x_val | back_right.x_val) < x_min){
    return false;
  }
  else if((front_left.y_val | front_right.y_val | back_left.y_val | back_right.y_val) > y_max){
    return false;
  }
  else if((front_left.y_val | front_right.y_val | back_left.y_val | back_right.y_val) < y_min){
    return false;
  }else{
    return true;
  }
}

void setOpenIntakeVals(){
 front_right.x_val = sp->x + (dist_cent_to_open_claw_x * cosdegrees(sp->theta)); //x and y distances from center of robot to claw extremity (need to recheck)
 front_left.x_val = sp->x - (dist_cent_to_open_claw_x * cosdegrees(sp->theta));//just change theta to 90+theta?
 front_right.y_val = sp->y + (dist_cent_to_open_claw_y * sindegrees(sp->theta));
 front_left.y_val = sp->y - (dist_cent_to_open_claw_y * sindegrees(sp->theta));
 f_rad = sqrt(dist_cent_to_open_claw_y * dist_cent_to_open_claw_y + dist_cent_to_open_claw_x * dist_cent_to_open_claw_x);
}

void setClosedIntakeVals(){
 front_right.y_val = front_left.y_val  = sp->y + (dist_cent_to_closed_claw * sindegrees(sp->theta));//since extremity points are together
 front_right.x_val = front_left.x_val = sp->x + (dist_cent_to_closed_claw * cosdegrees(sp->theta));
 f_rad = dist_cent_to_closed_claw;
}

#endif //COLLISIONHANDLER_C_INCLUDED
