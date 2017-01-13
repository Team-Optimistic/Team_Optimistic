#ifndef COLLISIONHANDLER_C_INCLUDED
#define COLLISIONHANDLER_C_INCLUDED

typedef struct statePack_t
{
  long x, y, theta;
} statePack;

long doesDriveCollide(const int mm)
{
  // Given our current state (x, y, theta, intake, lift),
  // If we drive straight for the given millimeters,
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where along our path do we hit it?
}

long doesDriveCollideSP(const statePack &sp, const int mm)
{
}

long doesTurnCollide(const int deg)
{
  // Given our current state (x, y, theta, intake, lift),
  // If we turn for the given degrees (clockwise),
  // Do we hit the field wall?
  // Do we hit the fence?
  // If we do hit something, where in the turn do we hit it?
}

long doesTurnCollideSP(const statePack &sp, const int mm)
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

long doesIntakeCollideSP(const statePack &sp, const int mm)
{
}

#endif //COLLISIONHANDLER_C_INCLUDED
