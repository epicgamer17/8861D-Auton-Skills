#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor backLeft = motor(PORT3, ratio6_1, false);
motor frontLeft = motor(PORT1, ratio6_1, true);
motor backRight = motor(PORT9, ratio6_1, false);
motor frontRight = motor(PORT8, ratio6_1, true);
motor intake = motor(PORT20, ratio6_1, false);
digital_out expansion = digital_out(Brain.ThreeWirePort.E);
controller Controller1 = controller(primary);
inertial inertialSensor = inertial(PORT11);
motor flyWheel = motor(PORT19, ratio18_1, false);
motor midLeft = motor(PORT2, ratio6_1, true);
motor midRight = motor(PORT10, ratio6_1, false);
encoder sidewaysRotation = encoder(Brain.ThreeWirePort.C);
encoder forwardRotation = encoder(Brain.ThreeWirePort.A);

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