using namespace vex;

extern brain Brain;

using signature = vision::signature;

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
extern signature visionSensor__YELLOW_DISC;
extern signature visionSensor__SIG_2;
extern signature visionSensor__SIG_3;
extern signature visionSensor__SIG_4;
extern signature visionSensor__SIG_5;
extern signature visionSensor__SIG_6;
extern signature visionSensor__SIG_7;
extern vision visionSensor;
extern controller Controller2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );