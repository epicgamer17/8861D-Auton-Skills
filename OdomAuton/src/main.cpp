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
// backLeft             motor         4               
// frontLeft            motor         2               
// backRight            motor         3               
// frontRight           motor         1               
// flyWheel1            motor         6               
// flyWheel2            motor         7               
// intake               motor         5               
// indexer              motor         8               
// expansion            digital_out   H               
// Controller1          controller                    
// inertialSensor       inertial      11              
// sidewaysRotation     rotation      18              
// forwardRotation      rotation      19              
// opticalSensor        optical       16              
// visionSensor         vision        20              
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
float wheelRadius = 0.041275;

bool stopIntake = true;

bool fieldOriented = false;

// #pragma endregion Odometry
#pragma endregion Variables

void aimToHighGoal() {
  turnToPoint(0.37, 3.25, 2500, false, true);
  waitUntil(enablePID == false);
}

void shootInHighGoal(int numShots = 1) {
  // turnToPoint(0.37, 3.25, 2500, false, true);
  // waitUntil(enablePID == false);
  aimToHighGoal();
  for (int i = 0; i < numShots; i ++) {
    index();
    wait(750, msec);
  }
  toggleFlyWheel();
}

float ptToPtDistance (float x1, float y1, float x2, float y2) {
    float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
    return dist;
  }


void goToNearestRoller() {
  if (ptToPtDistance(globalX, globalY, 0.525, 0.525) < ptToPtDistance(globalX, globalY, 3.075, 3.075)) {  
    if (ptToPtDistance(globalX, globalY, 0.15, 0.735) < ptToPtDistance(globalX, globalY, 0.735, 0.15)) {
      driveTo(0.3, 0.75, 0, 7500, 1);
      waitUntil(enablePID == false);
    } else {
      driveTo(0.85, 0.3, M_PI/2, 7500, 1);
      waitUntil(enablePID == false);
    }
  } else {
    if (ptToPtDistance(globalX, globalY, 2.9, 3.45) < ptToPtDistance(globalX, globalY, 3.45, 2.9)) {
      driveTo(2.85, 3.3, 3*M_PI/2, 7500, 1);
      waitUntil(enablePID == false);
    } else {
      driveTo(3.3, 2.75, M_PI, 7500, 1);
      waitUntil(enablePID == false);
    }
  }
}

void expand() {
  expansion.set(true);
}

int autonExpand() {
  wait (50, sec);
  expand();
  return 0;
}

void toggleFieldOriented() {
  fieldOriented = !fieldOriented;
  inertialSensor.resetHeading();
}

//Pre Autonomous
void pre_auton() {
  vexcodeInit();
  
  // inertialSensor.startCalibration();
  sidewaysRotation.resetPosition();
  forwardRotation.resetPosition();

  indexer.setBrake(brakeType::coast);
  indexer.setStopping(brakeType::coast);
  indexer.setTimeout(1, timeUnits::sec);
  flyWheel1.setStopping(brakeType::coast);
  flyWheel2.setStopping(brakeType::coast);
  flyWheel1.setBrake(brakeType::coast);
  flyWheel2.setBrake(brakeType::coast);

  frontLeft.setStopping(brakeType::coast);
  backLeft.setStopping(brakeType::coast);
  frontRight.setStopping(brakeType::coast);
  backRight.setStopping(brakeType::coast); 


  inertialSensor.calibrate();

  while(inertialSensor.isCalibrating()) {
    task::sleep(100);
  }
  ////DONT FORGET TO SET THE INERTIAL HEADING TO THE START HEADING 
  inertialSensor.setRotation(-90, deg);
  inertialSensor.setHeading(-90, deg);

  Brain.Screen.setCursor(3, 4);
  Brain.Screen.print("CALIBRATED!");



  // frontLeft.setBrake(brakeType::hold);
  // backLeft.setBrake(brakeType::hold);
  // frontRight.setBrake(brakeType::hold);
  // backRight.setBrake(brakeType::hold);

  // expansion.set(false);

  // task sPID = task(PID);
}

void autonSkillsV1() {

  ///CONSIDER USING DRIVE TO ROLLER FUNCTION FOR THIS
  driveTo(0.65, 0.3, M_PI/2, 2500, 1); //0.65 instead of 0.75 s that the optical sensor sees the roller
  waitUntil(enablePID == false);
  rollerBlue();
  driveTo(0.8, 0.4, M_PI/2, 2500, 1);
  waitUntil(enablePID == false);
  toggleIntake();
  turnToPoint(0.5, 0.7, 2500, true, false);
  waitUntil(enablePID == false);
  driveTo(0.6, 0.6, currentAbsoluteOrientation, 2500, 1); 
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(0.275, 0.85, 0, 2500, 1); //did 0.275 bc the bot always falls short, but 0.85 is so that optical can see the roller
  waitUntil(enablePID == false);
  rollerBlue();

  flyWheelSpeed = 8.7;
  toggleFlyWheel();
  driveTo(0.25, 2.05, 7*M_PI/16, 2500, 1);
  // waitUntil(enablePID == false);
  // turnToPoint(0.37, 3.25, 2500, false, true);
  // waitUntil(enablePID == false);
  // index();
  // wait(1000, msec);
  // index();
  // wait(1000, msec);
  // index();
  // wait(1000, msec);
  // toggleFlyWheel();
  shootInHighGoal(3);

  toggleFlyWheel();
  toggleIntake();
  driveTo(1.35, 2.05, 5*M_PI/8, 2500, 1);
  waitUntil(enablePID == false);
  // turnToPoint(0.37, 3.25, 2500, false, false);
  // wait(100, msec);
  // waitUntil(enablePID == false);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // toggleFlyWheel();
  shootInHighGoal(3);


  driveTo(1.525, 2.125, currentAbsoluteOrientation, 2500, 1);
  waitUntil(enablePID == false);
  driveTo(0.9, 1.5, M_PI/4, 2500, 1);
  waitUntil(enablePID == false);
  toggleFlyWheel();
  driveTo(1.4, 2.25, 3*M_PI/4, 2500, 1);
  // waitUntil(enablePID == false);
  
  // turnToPoint(0.37, 3.25, 2500, false, true);
  // waitUntil(enablePID == false);  
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  shootInHighGoal(3);


  toggleFlyWheel();
  driveTo(1.35, 3.2, M_PI/2, 2500, 1);
  waitUntil(enablePID == false);
  // turnToPoint(0.37, 3.25, 2500, false, false);
  // waitUntil(enablePID == false);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  shootInHighGoal(3);

  toggleFlyWheel();
  driveTo(1.8, 3.375, currentAbsoluteOrientation, 2500, 1);
  // waitUntil(enablePID == false);
  // turnToPoint(0.37, 3.25, 2500, false, true);
  // waitUntil(enablePID == false);  
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // toggleFlyWheel();
  shootInHighGoal(7);


  toggleFlyWheel();
  driveTo(2.15, 2.75, currentAbsoluteOrientation, 2500, 1);
  turnToPoint(2.2, 2.65, 2500, true, true);
  wait(1, sec);
  waitUntil(enablePID == false);

  // turnToPoint(0.37, 3.25, 2500, false, false);
  // waitUntil(enablePID == false);  
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  shootInHighGoal(3);

  toggleFlyWheel();
  driveTo(2.75, 2.75, M_PI, 2500, 1);
  waitUntil(enablePID == false);
  // turnToPoint(0.37, 3.25, 2500, false, false);
  // waitUntil(enablePID == false);  
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // index();
  // wait(750, msec);
  // toggleFlyWheel();
  shootInHighGoal(3);

  driveTo(2.85, 3.3, 3*M_PI/2, 2500, 1);
  waitUntil(enablePID == false);
  rollerBlue();
  driveTo(3.3, 2.85, M_PI, 2500, 1);
  waitUntil(enablePID == false);
  rollerBlue();
  driveTo(3, 3, 5*M_PI/4, 2500, 10);
  waitUntil(enablePID == false);
  expand();
  driveTo(0.85, 0.35, M_PI/2, 10000, 1);
  waitUntil(enablePID == false);
  enablePID = false;
}

//Autonomuos 
void autonomous(void) {  

  //reset rotation sensors
  forwardRotation.resetPosition();
  sidewaysRotation.resetPosition();

  frontLeft.resetRotation();
  frontRight.resetRotation();
  backLeft.resetRotation();
  backRight.resetRotation();
  
  // visionPickUpDisc();

  //start the odometry
  task odometryTask(positionTracking);
  // task IMUodometry(IMUPositionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  // task pidTask(PIDTask);
  task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  task failSafeExpansion(autonExpand);
  // task flyWheelTask(flyWheelPICTask);
  // visionPickUpDisc();


  // while (inertialSensor.rotation() < 1800) {
  //   frontLeft.spin(fwd);
  //   frontRight.spin(reverse);
  //   backLeft.spin(fwd);
  //   backRight.spin(reverse);
  // }

  // frontLeft.stop();
  // frontRight.stop();
  // backLeft.stop();
  // backRight.stop();
  
  ////////// AUTON SKILLS//////////
  //Start At Funnel? Shoot 7 Discs// 
  
  //First Roller// 
  driveTo(0.7, 0.31, M_PI/2, 2500, 1);
  waitUntil(enablePID == false);
  driveTo(0.7, 0.29, M_PI/2, 2500, 1);
  rollerBlue();
  waitUntil(enablePID == false);
  
  //Pick Up Disc at 0.6 0.6?//
  driveTo(1.2, 0.6, 0, 2500, 1);
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(0.6, 0.6, 0, 2500, 1);
  waitUntil(enablePID == false);
  ///
  ///

  //Second Roller//
  driveTo(0.31, 0.75, 0, 2500, 1);
  waitUntil(enablePID == false);
  driveTo(0.29, 0.75, 0, 2500, 1);
  rollerBlue();
  waitUntil(enablePID == false);

  //Shoot in High Goal//
  driveTo(0.6, 1.2, M_PI/2, 2500, 1);
  toggleFlyWheel();
  waitUntil(enablePID == false);
  shootInHighGoal(3);
  
  //Get More Discs While Going Towards Other Corner// 

  //Shoot in High Goal// 

  //Third Roller//

  //Forth Roller//

  //Expansion (Should Already Be Automatic)//
}

//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  task odometryTask(positionTracking);
  // task IMUodometry(IMUPositionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  // task flyWheelTask(flyWheelPICTask);
  
  enablePID = false;
  userControl = true;

  float backLeftSpeed = 0;
  float frontLeftSpeed = 0;
  float backRightSpeed = 0;
  float frontRightSpeed = 0;

  indexer.setBrake(brakeType::coast);
  indexer.setStopping(brakeType::coast);

  while (true) {
    int joystickAxis3 = Controller1.Axis3.value();
    int joystickAxis4 = Controller1.Axis4.value();
    int joystickAxis1 = Controller1.Axis1.value()  * 0.7;
  
    joystickAxis3 = abs(joystickAxis3) < 15 ? 0 : joystickAxis3;
    joystickAxis4 = abs(joystickAxis4) < 15 ? 0 : joystickAxis4;
    joystickAxis1 = abs(joystickAxis1) < 15 ? 0 : joystickAxis1;

    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("Field Oriented: %s", fieldOriented ? "on" : "off");
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("Fly Wheel Speed: %f", flyWheelSpeed);
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("----------------------");

    // printf("\n Brightnesss %f", opticalSensor.brightness());
    // printf("\n Hue %f", opticalSensor.hue());


    if (fieldOriented == true) {
      int turn = joystickAxis1;
      int x = joystickAxis4;
      int y = joystickAxis3;
      float inertialRadians = (inertialSensor.heading() - 45) * (M_PI/180) + M_PI/2; // + M_PI/2 Because odom has orientation as M_PI/2
      float temp = y * cos(inertialRadians) + x * sin(inertialRadians);
      x = - y * sin(inertialRadians) + x * cos(inertialRadians);
      y = temp;

      backLeftSpeed = (y + turn);
      frontLeftSpeed = (x + turn);
      backRightSpeed = (x - turn);
      frontRightSpeed = (y - turn);
    } else {
      backLeftSpeed = ((joystickAxis3 - joystickAxis4 + joystickAxis1));
      frontLeftSpeed = ((joystickAxis3 + joystickAxis4 + joystickAxis1));
      backRightSpeed = ((joystickAxis3 + joystickAxis4 - joystickAxis1));
      frontRightSpeed = ((joystickAxis3 - joystickAxis4 - joystickAxis1)); 
    }

    if (joystickAxis1 != 0 || joystickAxis3 != 0 || joystickAxis4 != 0) {
      enablePID = false;
      desiredX = globalX;
      desiredY = globalY;
      desiredHeading = currentAbsoluteOrientation;
      backLeft.spin(vex::directionType::fwd, backLeftSpeed, vex::velocityUnits::pct); //consider doing voltage
      frontLeft.spin(vex::directionType::fwd, frontLeftSpeed, vex::velocityUnits::pct);
      backRight.spin(vex::directionType::fwd, backRightSpeed, vex::velocityUnits::pct);
      frontRight.spin(vex::directionType::fwd, frontRightSpeed, vex::velocityUnits::pct);
    } else if (enablePID == false) {
      backLeft.stop(coast); //consider doing voltage
      frontLeft.stop(coast);
      backRight.stop(coast);
      frontRight.stop(coast);

    }
    if (Controller1.ButtonL2.pressing()) 
    {
      intake.spin(reverse, 100, velocityUnits::pct);    
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
  Controller1.ButtonR1.pressed(index);
  Controller1.ButtonL1.pressed(toggleIntake);
  //R2 Unused
  //L2 Used Above in User Control
  Controller1.ButtonUp.pressed(increaseFlyWheelSpeed);
  Controller1.ButtonDown.pressed(decreaseFlyWheelSpeed);
  Controller1.ButtonLeft.pressed(expand);
  Controller1.ButtonA.pressed(toggleFlyWheel);
  Controller1.ButtonB.pressed(toggleFieldOriented);
  
  Controller2.ButtonY.pressed(rollerBlue);
  Controller2.ButtonX.pressed(goToNearestRoller);
  Controller2.ButtonA.pressed(aimToHighGoal);

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