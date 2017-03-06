#ifndef POINTMATH_C_INCLUDED
#define POINTMATH_C_INCLUDED

#define ONE_TILE_MM   610
#define TWO_TILE_MM   1219
#define THREE_TILE_MM 1829
#define FOUR_TILE_MM  2438
#define FIVE_TILE_MM  3048

typedef struct distanceAndAngle_t
{
	float length;
	float theta;
} distanceAndAngle;

/**
 * Computes the distance to a point
 * @param  x X coordinate of point
 * @param  y Y coordinate of point
 * @return   Distance to point in mm
 */
float computeDistanceToPoint(const long x, const long y)
{
	BCI_lockSem(std_msgSem, "computeDistanceToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];
		return sqrt((xDiff * xDiff) + (yDiff * yDiff));

		BCI_unlockSem(std_msgSem, "computeDistanceToPoint")
	}

	//If no lock, return no distance
	return 0;
}

/**
 * Computes the angle to a point
 * @param  x X coordinate of point
 * @param  y Y coordinate of point
 * @return   Angle to point in degrees
 */
float computeAngleToPoint(const long x, const long y)
{
	BCI_lockSem(std_msgSem, "computeAngleToPoint")
	{
		//Compute difference in distance
		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];

		//Compute difference in angle
		return (atan2(yDiff, xDiff) * (180 / PI)) - std_msg[STD_MSG_EST_THETA];

		BCI_unlockSem(std_msgSem, "computeAngleToPoint")
	}

	//If no lock, return no angle
	return 0;
}

/**
 * Computes the disdistance and angle to a point
 * @param x   X coordinate of point
 * @param y   Y coordinate of point
 * @param out Struct for result
 */
void computeDistanceAndAngleToPoint(const long x, const long y, distanceAndAngle *out)
{
	//If no lock, return empty type
	out->length = 0;
	out->theta = 0;

	BCI_lockSem(std_msgSem, "computeDistanceAndAngleToPoint")
	{
		//Compute difference in distance
		#ifdef POINTMATH_DEBUG
			writeDebugStreamLine("comp: x: %d, estx: %d, y: %d, esty: %d, estth: %d", x, std_msg[STD_MSG_EST_X], y, std_msg[STD_MSG_EST_Y], std_msg[STD_MSG_EST_THETA]);
		#endif

		const float xDiff = x - std_msg[STD_MSG_EST_X], yDiff = y - std_msg[STD_MSG_EST_Y];
		out->length = sqrt((xDiff * xDiff) + (yDiff * yDiff));

		//Dont divide by 0
		if (xDiff < 0.0001 && xDiff > -0.0001)
		{
			const int ySgn = sgn(yDiff);

			if (ySgn == 1)
			{
				out->theta = -1 * std_msg[STD_MSG_EST_THETA];
			}
			else if (ySgn == -1)
			{
				out->theta = -180 - std_msg[STD_MSG_EST_THETA];

				if (out->theta <= -360)
					out->theta = out->theta + 360;
				else if (out->theta >= 360)
					out->theta = out->theta - 360;
			}
			else
			{
				out->theta = 0;
			}
		}
		else
		{
			//Compute difference in angle
			float temp = (atan2(yDiff, xDiff) * (180 / PI)) - std_msg[STD_MSG_EST_THETA];
			if (temp < 0)
				out->theta = 90 - temp;
			else
				out->theta = 90 + temp;
		}

		BCI_unlockSem(std_msgSem, "computeDistanceAndAngleToPoint")
	}
}

#endif //POINTMATH_C_INCLUDED
