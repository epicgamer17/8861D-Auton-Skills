#pragma region VEXcode Generated Robot Configuration

#include "vex.h"
#include "PID.h"
#include "draw-field.h"
#include "PPS.h"
#include "intake.h"
#include "flywheel.h"
// #include "IMUPositionTracking.h"


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// backLeft             motor         8               
// frontLeft            motor         6               
// backRight            motor         3               
// frontRight           motor         1               
// intake               motor         16              
// expansion            digital_out   E               
// Controller1          controller                    
// inertialSensor       inertial      18              
// flyWheel             motor         20              
// midLeft              motor         10              
// midRight             motor         5               
// sidewaysRotation     encoder       C, D            
// forwardRotation      encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----
// Robot configuration code.
#pragma endregion VEXcode Generated Robot Configuration

using namespace vex;

competition Competition; 

#pragma region Variables
// Variables 
float j = 0.0125/360; //Turning correction constant i think im not sure ask friend  
float tileLength = 0.4572; //  0.6096
float wheelRadius = 0.041275;

bool stopIntake = true;

#pragma endregion Variables

float ptToPtDistance (float x1, float y1, float x2, float y2) {
  float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
  return dist;
}

void expand() {
  expansion.set(true);
}

//Pre Autonomous
void pre_auton() {
  expansion.set(false);
  vexcodeInit();
  
  sidewaysRotation.resetRotation();
  forwardRotation.resetRotation();

  frontLeft.setStopping(brakeType::coast);
  backLeft.setStopping(brakeType::coast);
  frontRight.setStopping(brakeType::coast);
  backRight.setStopping(brakeType::coast); 


  inertialSensor.calibrate();

  while(inertialSensor.isCalibrating()) {
    task::sleep(100);
  }
  ////DONT FORGET TO SET THE INERTIAL HEADING TO THE START HEADING 
  inertialSensor.setRotation(-180, deg);
  inertialSensor.setHeading(-180, deg);



  Brain.Screen.setCursor(3, 4);
  Brain.Screen.print("CALIBRATED!");
}

//Autonomuos 
void autonomous(void) {  
  //reset rotation sensors
  forwardRotation.resetRotation();
  sidewaysRotation.resetRotation();

  frontLeft.resetRotation();
  frontRight.resetRotation();
  backLeft.resetRotation();
  backRight.resetRotation();
  
  //start the odometry
  task odometryTask(positionTracking);
  // task IMUodometry(IMUPositionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  // task failSafeExpansion(autonExpand);
  task flyWheelTask(flyWheelPICTask);
  // visionPickUpDisc();
  // wait(60, sec);
  // expansion.set(false);
  driveTo(60, 72, M_PI, 2500, 1);
  waitUntil(enablePID == false);
}

//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  task odometryTask(positionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  // task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  task flyWheelTask(flyWheelPICTask);

  desiredX = globalX;
  desiredY = globalY;
  enablePID = false;
  userControl = true;

  float leftSpeed = 0;
  float rightSpeed = 0;

  flyWheelSpeed = 480;
  waitUntil(enablePID == false);

  while (true) {
    int joystickAxis3 = Controller1.Axis3.value();
    int joystickAxis4 = Controller1.Axis4.value();
    int joystickAxis1 = Controller1.Axis1.value()  * 0.7;
  
    joystickAxis3 = abs(joystickAxis3) < 15 ? 0 : joystickAxis3;
    joystickAxis4 = abs(joystickAxis4) < 15 ? 0 : joystickAxis4;
    joystickAxis1 = abs(joystickAxis1) < 15 ? 0 : joystickAxis1;

    leftSpeed = ((joystickAxis3 + joystickAxis1))/8.333333;
    rightSpeed = ((joystickAxis3 - joystickAxis1))/8.333333;

    frontRight.spin(fwd, rightSpeed, voltageUnits::volt);
    midRight.spin(fwd, rightSpeed, voltageUnits::volt);
    backRight.spin(fwd, rightSpeed, voltageUnits::volt);
    
    frontLeft.spin(fwd, leftSpeed, voltageUnits::volt);
    midLeft.spin(fwd, leftSpeed, voltageUnits::volt);
    backLeft.spin(fwd, leftSpeed, voltageUnits::volt);


    if (Controller1.ButtonL2.pressing()) 
    {
      intake.spin(reverse, 40, velocityUnits::pct);    
      intakeOn = false;
    }
    else 
    {
      if (intakeOn == false)
      {
        intake.stop();
      }
    }
    //Don't hog the CPU
    // wait(20, msec);
  }
}

int main() {
  // Event Registration for Buttons
  Controller1.ButtonLeft.pressed(expand);
  Controller1.ButtonR1.pressed(toggleFlyWheel);
  Controller1.ButtonL1.pressed(toggleIntake);
  Controller1.ButtonUp.pressed(increaseFlyWheelSpeed);
  Controller1.ButtonDown.pressed(decreaseFlyWheelSpeed);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}