#ifndef COLLISIONHANDLER_C_INCLUDED
#define COLLISIONHANDLER_C_INCLUDED

typedef struct{
	int x_val;
	int y_val;
} claw_extremity;

claw_extremity front_left;
claw_extremity front_right;
claw_extremity back_left;
claw_extremity back_right;
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

bool testCornerCollision(){ //return true if a corner of the chassis hits the wall
	if((front_left.x_val | front_right.x_val | back_left.x_val | back_right.x_val) > x_max){
		return true;
	}
	else if((front_left.x_val | front_right.x_val | back_left.x_val | back_right.x_val) < x_min){
		return true;
	}
	else if((front_left.y_val | front_right.y_val | back_left.y_val | back_right.y_val) > y_max){
		return true;
	}
	else if((front_left.y_val | front_right.y_val | back_left.y_val | back_right.y_val) < y_min){
		return true;
		}else{
		return false;
	}
}

void setOpenIntakeVals(statePack *sp){
	front_right.x_val = sp->x + (dist_cent_to_open_claw_x * cosDegrees(sp->theta)); //x and y distances from center of robot to claw extremity (need to recheck)
	front_left.x_val = sp->x - (dist_cent_to_open_claw_x * cosDegrees(sp->theta));
	front_right.y_val = sp->y + (dist_cent_to_open_claw_y * sinDegrees(sp->theta));
	front_left.y_val = sp->y - (dist_cent_to_open_claw_y * sinDegrees(sp->theta));
	f_rad = sqrt(dist_cent_to_open_claw_y * dist_cent_to_open_claw_y + dist_cent_to_open_claw_x * dist_cent_to_open_claw_x);
}

void setClosedIntakeVals(statePack *sp){
	front_right.y_val = front_left.y_val  = sp->y + (dist_cent_to_closed_claw * sinDegrees(sp->theta));//since extremity points are together
	front_right.x_val = front_left.x_val = sp->x + (dist_cent_to_closed_claw * cosDegrees(sp->theta));
	f_rad = dist_cent_to_closed_claw;
}


void setCornerVals(statePack *sp){
	back_right.x_val = sp->x + (ft_to_mm * cosDegrees(sp->theta - 135)); //x and y distances from center of robot to back corners
	back_left.x_val = sp->x + (ft_to_mm * cosDegrees(sp->theta + 135));//minus?
	back_right.y_val = sp->y + (ft_to_mm * sinDegrees(sp->theta - 135 ));
	back_left.y_val = sp->y + (ft_to_mm * sinDegrees(sp->theta + 135));//minus?

	switch (intakeAndLiftTask_liftState){
	case LIFT_DOWN:

		switch (intakeAndLiftTask_intakeState)
		{
		case INTAKE_OPEN:
			setOpenIntakeVals(sp);
			break;
		case INTAKE_CLOSED:
			setClosedIntakeVals(sp);
			break;
		// case default :
		////////////////something, for when state is wait or rest
		// break;
	}
	break;
	case LIFT_UP:
	front_right.x_val = sp->x + (ft_to_mm * cosDegrees(sp->theta - 45)); //x and y distances from center of robot to back corners
	front_left.x_val = sp->x + (ft_to_mm * cosDegrees(sp->theta + 45));//minus?
	front_right.y_val = sp->y + (ft_to_mm * sinDegrees(sp->theta - 45));
	front_left.y_val = sp->y + (ft_to_mm * sinDegrees(sp->theta + 45));//minus?
	break;
}
}

long doesDriveCollideSP(const statePack *sp, const int mm)  {
	// Given our current state (x, y, theta, intake, lift),
	// If we drive straight for the given millimeters,
	// Do we hit the field wall?
	// Do we hit the fence?
	// If we do hit something, where along our path do we hit it?
	long s_dist;
	int x_dist = 0;
	int y_dist = 0;
setCornerVals(sp);

int front_right_goal_x = front_right.x_val + (mm * sinDegrees(sp->theta));
int front_right_goal_y = front_right.y_val + (mm * cosDegrees(sp->theta));
int front_left_goal_x = front_left.x_val - (mm * sinDegrees(sp->theta));
int front_left_goal_y = front_left.y_val - (mm * cosDegrees(sp->theta));
int back_right_goal_x = back_right.x_val + (mm * sinDegrees(sp->theta));
int back_right_goal_y = back_right.y_val + (mm * cosDegrees(sp->theta));
int back_left_goal_x = back_left.x_val - (mm * sinDegrees(sp->theta));
int back_left_goal_y = back_left.y_val - (mm * cosDegrees(sp->theta));

//could also put in a while loop and use testCornerCollision like turning--probably slower
	if((front_left_goal_x > x_max) || (front_left_goal_y > y_max)){
		x_dist = x_max - front_left.x_val;//dist from current location to max point
		y_dist = y_max - front_left.y_val;
	}
	if((front_left_goal_x < x_min) || (front_left_goal_y < y_min)){
		x_dist = front_left.x_val - x_min;//dist from current location to max point
		y_dist = front_left.y_val - y_min;
	}
	if((front_right_goal_x > x_max)|| (front_right_goal_y > y_max)){
		x_dist = x_max - front_right.x_val;
		y_dist = y_max - front_right.y_val;
	}
	if((front_right_goal_x < x_min)|| (front_right_goal_y < y_min)){
		x_dist = front_right.x_val - x_min;
		y_dist = front_right.y_val - y_min;
	}
	if((back_left_goal_x > x_max)||(back_left_goal_y > y_max)){
		x_dist = x_max - back_left.x_val;//dist from current location to max point
		y_dist = y_max - back_left.y_val;
	}
	if((back_left_goal_x < x_min)||(back_left_goal_y < y_min)){
		x_dist = back_left.x_val - x_min;//dist from current location to max point
		y_dist = back_left.y_val - y_min;
	}
	if((back_right_goal_x > x_max) || (back_right_goal_y > y_max)){
		x_dist = x_max - back_right.x_val;
		y_dist = y_max - back_right.y_val;
	}
	if((back_right_goal_x < x_min) || (back_right_goal_y < y_min)){
		x_dist = back_right.x_val - x_min;
		y_dist = back_right.y_val - y_min;
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
	setCornerVals(sp);////////////////////////////////////////////////////////////////////////
	long turndeg = 0;
	while(!testCornerCollision() && (turndeg <= deg)){
		front_left.x_val += f_rad * cos(turndeg + sp->theta);
		front_right.x_val += f_rad * cos(turndeg + sp->theta);
		front_left.y_val += f_rad * sin(turndeg + sp->theta);
		front_right.y_val += f_rad * sin(turndeg + sp->theta);
		back_left.x_val += ft_to_mm * cos(turndeg + sp->theta);
		back_right.x_val += ft_to_mm * cos(turndeg + sp->theta);
		back_left.y_val += ft_to_mm * sin(turndeg + sp->theta);
		back_right.y_val += ft_to_mm * sin(turndeg + sp->theta);
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

long doesindegreestakeCollideSP(const statePack *sp, const int mm)//what the fuck is this even supposed to be
																																	//it should be degrees intake is open collide?
{
setCornerVals(sp);
long wtf = 666;
return wtf;
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
#endif //COLLISIONHANDLER_C_INCLUDED
