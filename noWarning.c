void noWarning(){
		uart_verifyMessageCount(5,5);
		sendSPCMsg();
		sendMPCMsg();
		computeDistanceToPoint(5,5);
		computeAngleToPoint(5,5);
		int fake = dist_cent_to_closed_claw;
		fake = dist_cent_to_open_claw_x;
		fake = dist_cent_to_open_claw_y;
		doesDriveCollide(5);
		doesTurnCollide(5);
		doesindegreestakeCollide(5);
		moveToPoint_Translate(5,5,true);
		scoreFence(FENCE_LEFT);
		pickUpStars(&fake,&fake);
		pickUpCube(5,5);
		cheeseThoseStars();



		if(false)
			noWarning();
}
