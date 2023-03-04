#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor backLeft = motor(PORT12, ratio18_1, false);
motor frontLeft = motor(PORT11, ratio18_1, false);
motor backRight = motor(PORT19, ratio18_1, true);
motor frontRight = motor(PORT20, ratio18_1, true);
motor intake = motor(PORT1, ratio18_1, false);
motor indexer = motor(PORT14, ratio18_1, true);
digital_out expansion = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
inertial inertialSensor = inertial(PORT15);
rotation sidewaysRotation = rotation(PORT9, false);
rotation forwardRotation = rotation(PORT8, true);
optical opticalSensor = optical(PORT18);
/*vex-vision-config:begin*/
signature visionSensor__YELLOW_DISC = signature (1, -237, 127, -55, -4455, -1861, -3158, 1.7, 0);
vision visionSensor = vision (PORT16, 50, visionSensor__YELLOW_DISC);
/*vex-vision-config:end*/
controller Controller2 = controller(partner);
motor flyWheel1d = motor(PORT6, ratio6_1, true);
motor flyWheel2d = motor(PORT7, ratio6_1, false);
distance distanceSensorBack = distance(PORT10);
distance distanceSensorRight = distance(PORT17);
distance distanceSensorLeft = distance(PORT13);
motor flyWheelMotorA = motor(PORT2, ratio6_1, true);
motor flyWheelMotorB = motor(PORT3, ratio6_1, false);
motor_group flyWheel = motor_group(flyWheelMotorA, flyWheelMotorB);
distance discCounter = distance(PORT4);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}