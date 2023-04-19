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
// backLeft             motor         3               
// frontLeft            motor         1               
// backRight            motor         9               
// frontRight           motor         8               
// intake               motor         20              
// expansion            digital_out   E               
// Controller1          controller                    
// inertialSensor       inertial      11              
// flyWheel             motor         19              
// midLeft              motor         2               
// midRight             motor         10              
// sidewaysRotation     encoder       C, D            
// forwardRotation      encoder       A, B            
// angleAdjuster        digital_out   F               
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
bool angleAdjusterBool = false;
#pragma endregion Variables

float ptToPtDistance (float x1, float y1, float x2, float y2) {
  float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
  return dist;
}

void expand() {
  expansion.set(true);
}

void angleAdjust() {
  angleAdjuster.set(!angleAdjusterBool);
  angleAdjusterBool = !angleAdjusterBool;
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
  inertialSensor.setRotation(-180, deg); //negative values
  inertialSensor.setHeading(-180, deg); //negative values



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
  // task flyWheelTask(flyWheelPICTask); //comment out if not using toggleFlywheel();
  // visionPickUpDisc();

  //REMEMBER TO SET YOUR GLOBAL X AND GLOBAL Y START AND TO SET YOUR GLOBAL HEADING 
  //GLOBAL HEADING NEEDS TO BE SET IN BOTH odometry.h AND here in the main in the preautonomous

  //TURN PID TEST
  //   turnTo(3*M_PI/4,2500);

  // waitUntil(enablePID==false);
  // turnTo(M_PI/2,2500);
  // // turnToPoint(20, 124, 5000);
  // waitUntil(enablePID==false);
  // turnTo(M_PI,2500);
  // // turnToPoint(20, 124, 5000);
  // waitUntil(enablePID==false);
  // turnTo(0,2500);
  // // turnToPoint(20, 124, 5000);
  // waitUntil(enablePID==false);



  // turnTo(M_PI/2, 5000); //this format turns to a global heading
  // waitUntil(enablePID==false);  //this format turns to a global heading
  // driveTo(12, 5000, 1.0); //this format uses relative to robot current positoin
  // waitUntil(enablePID==false); //this format uses relative to robot current positoin
  // wait(1000, msec); //this format uses relative to robot current positoin
  // driveTo(-12, 5000, 1.0); //this format uses relative to robot current positoin
  // waitUntil(enablePID==false); //this format uses relative to robot current positoin
  //turnToPoint(24, 72, 5000); //this format uses the global position
  //waitUntil(enablePID==false); //this format uses the global position
  //driveTo(ptToPtDistance(globalX, globalY, 24, 72), 2500, 1.0); //this format uses the global position
  //waitUntil(enablePID==false);//this format uses the global position
  
driveTo(-7, 1000, 1);
waitUntil(enablePID==false);
intake.spinFor(forward, 90, degrees);
driveTo(7, 1000, 1);
waitUntil(enablePID==false);
turnTo(3*M_PI/2, 1200);
waitUntil(enablePID==false);
driveTo(51, 2500, 1);
waitUntil(enablePID==false);
turnTo(M_PI, 1200);
waitUntil(enablePID==false);
toggleIntake();
driveTo(44.5, float timeoutTime, float mSpeed)
waitUntil(enablePID==false);


  
  //toggleIntake();
  //driveTo(48, 3000, 0.5);
  //flyWheel.spin(fwd, 12, voltageUnits::volt);
  //waitUntil(enablePID==false);
  //turnToPoint(27, 126, 2500);
  //waitUntil(enablePID==false);
  //driveTo(10, 3200, 0.5);
  //toggleIntake();
  //waitUntil(enablePID==false);
  //singleIndex();
  //wait(1300, msec);
  //singleIndex();
  //wait(1400, msec);
  //singleIndex();
}

//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  task odometryTask(positionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  // task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  task flyWheelTask(flyWheelPICTask);

  enablePID = false;

  float leftSpeed = 0;
  float rightSpeed = 0;

  while (true) {
    int joystickAxis3 = Controller1.Axis3.value();
    int joystickAxis4 = Controller1.Axis4.value();
    int joystickAxis1 = Controller1.Axis1.value()  * 0.6;
  
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


    if (Controller1.ButtonR2.pressing()) 
    {
      intake.spin(reverse, 90, velocityUnits::pct);
      wait(360, msec);
      intake.stop();
      wait(300, msec);
      intake.spin(reverse, 90, velocityUnits::pct);
      wait(250, msec);

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
  Controller1.ButtonY.pressed(expand);
  Controller1.ButtonL1.pressed(toggleFlyWheel);
  Controller1.ButtonA.pressed(toggleIntake);
  Controller1.ButtonUp.pressed(increaseFlyWheelSpeed);
  Controller1.ButtonDown.pressed(decreaseFlyWheelSpeed);
  Controller1.ButtonB.pressed(angleAdjust);
  Controller1.ButtonR1.pressed(singleIndex);

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