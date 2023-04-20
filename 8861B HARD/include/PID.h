#include "vex.h"
#include "odometry.h"

extern bool fieldOriented;
extern bool correctingPosition;
// extern float desiredY;
// extern float desiredX;
extern float desiredTurnY;
extern float desiredTurnX;
extern float desiredHeading;
extern float desiredForwardValue;
extern float driveError;
extern double drivePowerPID;
extern double turnPowerPID;
extern double turnError;

extern bool enablePID;
extern bool userControl;
extern bool turningToPoint;
extern bool invertedTurning;

extern void driveTo(float dFwd, float timeoutTime, float mSpeed);

extern void turnTo(float dH, float timeoutTime);

extern void turnToPoint(float dX, float dY, float timeoutTime);

void drivePID();
void turnPID();

int PIDTask();