#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor backLeft = motor(PORT4, ratio18_1, false);
motor frontLeft = motor(PORT2, ratio18_1, false);
motor backRight = motor(PORT3, ratio18_1, true);
motor frontRight = motor(PORT1, ratio18_1, true);
motor flyWheel1 = motor(PORT6, ratio6_1, true);
motor flyWheel2 = motor(PORT7, ratio6_1, false);
motor intake = motor(PORT5, ratio18_1, false);
motor indexer = motor(PORT8, ratio18_1, true);
digital_out expansion = digital_out(Brain.ThreeWirePort.H);
controller Controller1 = controller(primary);
inertial inertialSensor = inertial(PORT11);
rotation sidewaysRotation = rotation(PORT18, false);
rotation forwardRotation = rotation(PORT19, true);

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