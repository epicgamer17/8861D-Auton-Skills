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

void Auton() {
      intakeOn = true;
      printf("\n %f", flyWheelError);
      waitUntil(fabs(flyWheelError) < 10);
      intake.spinFor(175, msec, -90, velocityUnits::pct);
      intake.spinFor(200, msec, 20, velocityUnits::pct);
      wait(1400, msec);
      waitUntil(fabs(flyWheelError) < 10);
      intake.spinFor(200, msec, -90, velocityUnits::pct);
      intake.spinFor(200, msec, 20, velocityUnits::pct);
      wait(1400, msec);
      waitUntil(fabs(flyWheelError) < 10);
      intake.spinFor(500, msec, -90, velocityUnits::pct);
      intake.spinFor(200, msec, 20, velocityUnits::pct);
      intakeOn = false;
}

void burstShot() {
      if(fabs(flyWheelError) < 12){
        intakeOn = true;
        intake.spinFor(300, msec, -90, velocityUnits::pct);
        intake.spinFor(300, msec, 20, velocityUnits::pct);
        intake.spinFor(400, msec, -90, velocityUnits::pct);
        intakeOn = false;
      }
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
  inertialSensor.setRotation(-90, deg); //negative values
  inertialSensor.setHeading(-90, deg); //negative values



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
  task flyWheelTask(flyWheelPICTask); //comment out if not using toggleFlywheel();
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

// flyWheel.spin(fwd, 12, voltageUnits::volt);
// wait(3400, msec);
// intake.spinFor(225, msec, -95, velocityUnits::pct);
// wait(1800, msec);
// intake.spinFor(250, msec, -95, velocityUnits::pct);
// wait(1800, msec);
// flyWheel.spin(fwd, 11, voltageUnits::volt);
// intake.spinFor(300, msec, -95, velocityUnits::pct);
// flyWheel.stop();


// flyWheel.spin(fwd, 85, velocityUnits::pct);
// wait(4000, msec);
// intake.spin(reverse, 20, velocityUnits::pct);
// wait(400, msec);
// intake.spin(fwd, 20, velocityUnits::pct);
// wait(480, msec);
// intake.spin(reverse, 20, velocityUnits::pct);
// wait(450, msec);
// intake.spin(fwd, 20, velocityUnits::pct);
// wait(480, msec);
// intake.spin(reverse, 90, velocityUnits::pct);
// wait(900, msec);
// flyWheel.stop();
// intake.stop();


// Autonomous
toggleFlyWheel();
driveTo(-5, 300, 1);
waitUntil(enablePID==false);
intake.setVelocity(90, percent);
intake.spinFor(forward, 300, degrees);
driveTo(5, 1000, 2);
toggleIntake();
waitUntil(enablePID==false);
driveTo(10, 10000, 1);
waitUntil(enablePID==false);
turnToPoint(28, 140, 1000);
waitUntil(enablePID==false);
wait(1000, msec); //to make the intake spin longer
toggleIntake();
flyWheelSpeed = 560;
Auton();
flyWheel.stop();
intake.stop();
driveTo(-9, 3000, 1); //made 750 from 800 - jonathan
waitUntil(enablePID==false);
turnToPoint(74.6, 48.5, 1000); //made 650 from 700 - jonathan 
waitUntil(enablePID==false);
toggleIntake();
flyWheel.spin(fwd, 11, voltageUnits::volt);
driveTo(50, 3000, 0.25);
waitUntil(enablePID==false);
turnToPoint(27, 126, 1000);
waitUntil(enablePID==false);
toggleIntake();
Auton();
flyWheel.stop();
intake.stop();

// toggleFlyWheel();
// driveTo(-5, 300, 1);
// waitUntil(enablePID==false);
// intake.setVelocity(90, percent);
// intake.spinFor(forward, 300, degrees);
// driveTo(5, 400, 1);
// toggleIntake();
// waitUntil(enablePID==false);
// driveTo(15.5, 2000, 1);
// waitUntil(enablePID==false);
// turnToPoint(30, 140, 400);
// waitUntil(enablePID==false);
// wait(200, msec); //to make the intake spin longer
// toggleIntake();
// flyWheelSpeed = 525;
// burstShot();
// flyWheel.stop();
// intake.stop();
// driveTo(-15, 750, 1); //made 750 from 800 - jonathan
// waitUntil(enablePID==false);
// turnToPoint(74.6, 48.5, 650); //made 650 from 700 - jonathan 
// waitUntil(enablePID==false);
// toggleIntake();
// flyWheel.spin(fwd, 11, voltageUnits::volt);
// driveTo(50, 3500, 0.7);
// waitUntil(enablePID==false);
// turnToPoint(27, 126, 800);
// waitUntil(enablePID==false);
// toggleIntake();
// burstShot();
// flyWheel.stop();
// intake.stop();
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


    // if (Controller1.ButtonR1.pressing()) 
    // {

    //   // intake.spinFor(200, msec, -90, velocityUnits::pct);
    //   // intake.spinFor(200, msec, 20, velocityUnits::pct);
    //   // intake.spinFor(200, msec, -90, velocityUnits::pct);
    //   // intake.spinFor(500, msec, 20, velocityUnits::pct);
    //   // intake.spinFor(400, msec, -90, velocityUnits::pct);
    // }
    // else 
    // {
    if (Controller1.ButtonL2.pressing()) 
    {
      intake.spin(fwd, 50, velocityUnits::pct);
    }
    else 
    {
      if (intakeOn == false)
      {
        intake.stop();
      }
      
    }
    // }
    //Don't hog the CPU
    // wait(20, msec);
  }
}

int main() {
  // Event Registration for Buttons
  Controller1.ButtonY.pressed(expand);
  Controller1.ButtonL1.pressed(toggleFlyWheel);
  Controller1.ButtonA.pressed(toggleIntake);
  Controller1.ButtonB.pressed(angleAdjust);
  Controller1.ButtonR2.pressed(singleIndex);
  Controller1.ButtonR1.pressed(burstShot);
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