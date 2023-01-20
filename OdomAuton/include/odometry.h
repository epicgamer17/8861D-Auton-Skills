#include "vex.h"

//The current angle of the bot (RADIANS)
extern float currentAbsoluteOrientation;

//The global position of the bot (INCHES)
extern float globalX;
extern float globalY;

//The odometry function
int positionTracking();