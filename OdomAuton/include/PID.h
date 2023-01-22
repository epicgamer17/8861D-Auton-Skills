#include "vex.h"
#include "odometry.h"

extern float desiredY;
extern float desiredX;
extern float desiredHeading;

extern bool enablePID;

extern void driveTo(float dX, float dY, float dH, float timeoutTime, float mSpeed);

extern void driveToAndTurnToPoint(float dX, float dY, float timeoutTime, float mSpeed);

extern void turnTo(float dH, float timeoutTime);

extern void turnToPoint(float dX, float dY, float timeoutTime);

void setDrivePower(float theta);

void drivePID();
void turnPID();

int PIDTask();