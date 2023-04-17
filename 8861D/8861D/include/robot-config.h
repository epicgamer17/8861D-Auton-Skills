using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor backLeft;
extern motor frontLeft;
extern motor backRight;
extern motor frontRight;
extern motor intake;
extern digital_out expansion;
extern controller Controller1;
extern inertial inertialSensor;
extern motor flyWheel;
extern motor midLeft;
extern motor midRight;
extern encoder sidewaysRotation;
extern encoder forwardRotation;
extern digital_out adjust;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );