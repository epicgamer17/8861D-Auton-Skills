#pragma region VEXcode Generated Robot Configuration

#include "vex.h"
#include "PID.h"
#include "draw-field.h"
#include "PPS.h"
#include "intake.h"
#include "flywheel.h"
#include "drive.h"
// #include "IMUPositionTracking.h"


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// backLeft             motor         10              
// frontLeft            motor         6               
// backRight            motor         5               
// frontRight           motor         1               
// intake               motor         16              
// string               digital_out   H               
// Controller1          controller                    
// inertialSensor       inertial      18              
// flyWheel             motor         20              
// midLeft              motor         8               
// midRight             motor         3               
// sidewaysRotation     encoder       C, D            
// forwardRotation      encoder       A, B            
// angleAdjuster        digital_out   G               
// Controller2          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
// Robot configuration code.
#pragma endregion VEXcode Generated Robot Configuration

using namespace vex;

competition Competition; 

#pragma region Variables
// Variables 
float j = 0.0125/360; //Turning correction constant i think im not sure ask friend  
float tileLength = 0.4572; //  0.6096
// float wheelRadius = 0.041275;

bool stopIntake = true;
bool angleAdjusterBool = false;
#pragma endregion Variables

float ptToPtDistance (float x1, float y1, float x2, float y2) {
  float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
  return dist;
}

void expand() {
  string.set(true);
}

void angleAdjust() {
  angleAdjuster.set(!angleAdjusterBool);
  angleAdjusterBool = !angleAdjusterBool;
}

//Pre Autonomous
void pre_auton() {
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

  //EzySide

  
  
  //driveTo(20, 1000, 1);
  //waitUntil(enablePID==false);
  turnTo(2.7, 700);
  waitUntil(enablePID==false);
  driveTo(20,2000,1);
  
  Drive::Shots(600, 1308, 690);
  // driveTo(-12.5, 3000, 1);
  // waitUntil(enablePID == false);
  // turnTo(5.49779, 2000); 
  // waitUntil(enablePID == false); 
  // intake.startRotateFor(fwd, 5000, deg, 100, velocityUnits::pct);
  // driveTo(65, 3000, 1);
  // waitUntil(enablePID == false);
  // turnTo(M_PI/4, 2000);
  // waitUntil(enablePID == false);
  // driveTo(12, 2000, 1);
  // waitUntil(enablePID == false);
  // Drive::Shots(600, 540,690);
  // turnTo(M_PI/4, 2000);
  // waitUntil(enablePID == false);
  // driveTo(-100,3000,1);
  // waitUntil(enablePID == false);
  // intake.rotateFor(-600, deg, 600, velocityUnits::rpm);

  // Drive::Shots(600, 1400, 600);
  // wait(100, msec);
  // turnTo(0.5, 700);
  // waitUntil(enablePID==false);
  // intake.startRotateFor(fwd, -442000, deg, 600, velocityUnits::rpm);
  // driveTo(26, 1200, 1.5);
  // waitUntil(enablePID==false);
  // driveTo(29, 1400, 0.3);
  // waitUntil(enablePID==false);
  // flyWheel.startRotateFor(fwd,100000,deg,580,velocityUnits::rpm);
  // turnTo(M_PI_2, 500);
  // waitUntil(enablePID==false);
  //turnTo(1.9, 2000);
  // turnToPoint(20, 124, 2000);
  // waitUntil(enablePID==false);
  // turnTo(currentAbsoluteOrientation - M_PI/16, 2000);
  // waitUntil(enablePID==false);
  // driveTo(3, 1000, 1);
  // waitUntil(enablePID==false);
  // Drive::Shots(570, 100000, 690);

// Old Hard side
//   driveTo(-21, 1000, 1.5);
//   waitUntil(enablePID==false);
//   turnTo(M_PI, 1000);
//   waitUntil(enablePID==false);
//   driveTo(-7.4, 1000, 1.0);
//   waitUntil(enablePID==false);
//   intake.spinFor(fwd, -600 ,deg, 600, velocityUnits::rpm);
//   driveTo(5, 1000, 1.5);
//   waitUntil(enablePID==false);
//   turnTo(2.38, 1000);
//   waitUntil(enablePID==false);
//   intake.startRotateFor(fwd, -3000, deg, 600, velocityUnits::rpm);
//   driveTo(8, 1000, 1);
//   waitUntil(enablePID==false);
//   turnTo(2.8, 2000);
//   waitUntil(enablePID==false);
//   Drive::Shots(12, 1420);
//   driveTo(-8, 1000, 1);
//   waitUntil(enablePID==false);
//   turnTo(4.01426, 1420);
//   waitUntil(enablePID==false);
//   intake.startRotateFor(fwd, -54200, deg, 600, velocityUnits::rpm);
//   driveTo(70, 2000, 1);
//   waitUntil(enablePID==false);

//  Other stuff 

//   turnTo(3*M_PI/2,5000);
//   waitUntil(enablePID==false);
//   turnTo(M_PI,5000);
//   waitUntil(enablePID==false);
//   driveTo(12, 5000, 1.0);
//   waitUntil(enablePID==false);
//   driveTo(-12, 5000, 1.0);
//   waitUntil(enablePID==false);

//   turnTo(M_PI/2, 5000); //this format turns to a global heading
//   waitUntil(enablePID==false);  //this format turns to a global heading
//   driveTo(12, 5000, 1.0); //this format uses relative to robot current positoin
//   waitUntil(enablePID==false); //this format uses relative to robot current positoin
//   wait(1000, msec); //this format uses relative to robot current positoin
//   driveTo(-12, 5000, 1.0); //this format uses relative to robot current positoin
//   waitUntil(enablePID==false); //this format uses relative to robot current positoin
//   turnToPoint(24, 72, 5000); //this format uses the global position
//   waitUntil(enablePID==false); //this format uses the global position
//   driveTo(ptToPtDistance(globalX, globalY, 24, 72), 2500, 1.0); //this format uses the global position
//   waitUntil(enablePID==false);//this format uses the global position
  
// driveTo(-7, 1000, 1);
// waitUntil(enablePID==false);
// intake.spinFor(forward, 90, degrees);
// driveTo(7, 1000, 1);
// waitUntil(enablePID==false);
// turnTo(3*M_PI/2, 1200);
// waitUntil(enablePID==false);
// driveTo(51, 2500, 1);
// waitUntil(enablePID==false);
// turnTo(M_PI, 1200);
// waitUntil(enablePID==false);
// toggleIntake();
// driveTo(44.5, 2500, 1);
// waitUntil(enablePID==false);
  
//   toggleIntake();
//   driveTo(48, 3000, 0.5);
//   flyWheel.spin(fwd, 12, voltageUnits::volt);
//   waitUntil(enablePID==false);
//   turnToPoint(27, 126, 2500);
//   waitUntil(enablePID==false);
//   driveTo(10, 3200, 0.5);
//   toggleIntake();
//   waitUntil(enablePID==false);
//   singleIndex();
//   wait(1300, msec);
//   singleIndex();
//   wait(1400, msec);
//   singleIndex();
}



//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  task odometryTask(positionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  // task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);

  thread webs(Drive::expansion);
  thread thingy(Drive::updateController2);

  enablePID = false;

  float leftSpeed = 0;
  float rightSpeed = 0;

  while (true) {
    Drive::RobotOriented();
    Drive::Intake();
    wait (5,msec);

  }
}

int main() {
  // Event Registration for Buttons
  Controller1.ButtonL1.pressed(Drive::Shoot);
  Controller1.ButtonX.pressed(Drive::flappy);
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