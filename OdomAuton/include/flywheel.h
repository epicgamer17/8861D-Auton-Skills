#include "vex.h"

extern int flyWheelSpeed;

extern bool flyWheelOn;

// double flyWheelMaxError;

// double flyWheelIntegralBound;

// double flyWheelkP;
// double flyWheelkI;
// double flyWheelkD;

// double flyWheelPowerPID;

// float flyWheelError; //Desired Value - Sensor Value: Position
// float flyWheelPrevError; //Position 20ms ago
// float flyWheelDerivative; // error - prevError : Speed
// float flyWheelIntegral; // totalError += error : Integral 

//Index Function
extern void index();

extern void increaseFlyWheelSpeed();

extern void decreaseFlyWheelSpeed();

extern void flyWheelPID();

extern int flyWheelPIDTask();

extern void toggleFlyWheel();
