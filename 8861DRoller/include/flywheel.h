#include "vex.h"
#include "odometry.h"
#include "PID.h"

extern float flyWheelSpeed;
extern float flyWheelError;

extern bool flyWheelOn;

extern void increaseFlyWheelSpeed();

extern void decreaseFlyWheelSpeed();

extern int flyWheelPICTask();

int sgn (float num);

extern void toggleFlyWheel();
