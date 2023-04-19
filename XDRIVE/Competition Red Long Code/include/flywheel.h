#include "vex.h"
#include "odometry.h"
#include "PID.h"

extern float flyWheelSpeed;
extern float flyWheelError;

extern bool flyWheelOn;

//Index Function
extern void index();

extern void increaseFlyWheelSpeed();

extern void decreaseFlyWheelSpeed();

extern int flyWheelPICTask();

int sgn (float num);

extern int flyWheelTBHTask();

extern void toggleFlyWheel();
