using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor backLeft;
extern motor frontLeft;
extern motor backRight;
extern motor frontRight;
extern motor flyWheel1;
extern motor flyWheel2;
extern motor intake;
extern motor indexer;
extern digital_out expansion;
extern controller Controller1;
extern inertial inertialSensor;
extern rotation sidewaysRotation;
extern rotation forwardRotation;
extern optical opticalSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );