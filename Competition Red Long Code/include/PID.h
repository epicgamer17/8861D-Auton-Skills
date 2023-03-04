#include "vex.h"
#include "odometry.h"

extern bool fieldOriented;
extern bool correctingPosition;
extern float desiredY;
extern float desiredX;
extern float desiredTurnY;
extern float desiredTurnX;
extern float desiredHeading;

extern bool enablePID;
extern bool userControl;
extern bool turningToPoint;
extern bool invertedTurning;

extern void correctPosition(int calibrationHeading);

extern void driveTo(float dX, float dY, float dH, float timeoutTime, float mSpeed, bool distanceSensorBool);

extern void driveToAndTurnToPoint(float dX, float dY, float timeoutTime, float mSpeed);

extern void turnTo(float dH, float timeoutTime);

extern void turnToPoint(float dX, float dY, float timeoutTime, bool inverted, bool driving);

void setDrivePower(float theta);

void drivePID();
void turnPID();

int PIDTask();