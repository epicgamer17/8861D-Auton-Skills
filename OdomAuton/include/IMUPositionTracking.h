#include "vex.h"

//The current angle of the bot (RADIANS)
extern float IMUCurrentAbsoluteOrientation;

//The global position of the bot (METERS)
extern float IMUGlobalX;
extern float IMUGlobalY;

//The odometry function
int IMUPositionTracking();