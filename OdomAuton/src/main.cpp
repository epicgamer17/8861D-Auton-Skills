#pragma region VEXcode Generated Robot Configuration

#include "vex.h"
#include "PID.h"
#include "draw-field.h"
#include "PPS.h"
#include "intake.h"
#include "flywheel.h"

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


void expand() {
  expansion.set(true);
}

int autonExpand() {
  wait (50, sec);
  expand();
  return 0;
}

void controlSwitch() {
  fieldOriented = !fieldOriented;
  inertialSensor.resetHeading();
}

//Pre Autonomous
void pre_auton() {
  vexcodeInit();
  
  // inertialSensor.startCalibration();
  sidewaysRotation.resetPosition();
  forwardRotation.resetPosition();

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


//Autonomuos 
void autonomous(void) {  

  //reset rotation sensors
  // LTrack.resetPosition();
  forwardRotation.resetPosition();
  sidewaysRotation.resetPosition();

  frontLeft.resetRotation();
  frontRight.resetRotation();
  backLeft.resetRotation();
  backRight.resetRotation();

  //start the odometry
  task odometryTask(positionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  task failSafeExpansion = vex::task(autonExpand);
  // task flyWheelTask(flyWheelPIDTask);
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
  // turnTo(M_PI/2, 2500);
  // waitUntil(enablePID == false);

  driveTo(0.85, 0.15, M_PI/2, 2500, 2);
  waitUntil(enablePID == false);
  driveTo(0.85, 0.4, M_PI/2, 2500, 6);
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(0.6, 0.6, currentAbsoluteOrientation, 2500, 2); 
  turnToPoint(0.6, 0.6, 2500, true);
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(0.15, 0.85, 0, 2500, 2);
  waitUntil(enablePID == false);
  flyWheelSpeed = 9;
  toggleFlyWheel();
  driveTo(0.2, 2.25, 7*M_PI/16, 2500, 2);
  waitUntil(enablePID == false);
  index();
  wait(1000, msec);
  index();
  wait(1000, msec);
  index();
  wait(1000, msec);
  toggleFlyWheel();
  driveTo(0.2, 2.2, M_PI, 2500, 2);
  waitUntil(enablePID == false);
  toggleFlyWheel();
  driveTo(1.35, 2.3, M_PI, 2500, 2);
  waitUntil(enablePID == false);
  turnToPoint(0.37, 3.25, 2500, false);
  wait(1.5, sec); // dont know why for some reason it only works if this wait is here. 
  waitUntil(enablePID == false);
  index();
  wait(1000, msec);
  index();
  wait(1000, msec);
  index();
  wait(1000, msec);
  toggleFlyWheel();
  driveTo(2.75, 3.45, 3*M_PI/2, 5000, 1);
  waitUntil(enablePID == false);
  driveTo(3.45, 2.75, M_PI, 2500, 2);
  waitUntil(enablePID == false);
  driveTo(3, 3, 5*M_PI/4, 2500, 10);
  waitUntil(enablePID == false);
  expand();
  driveTo(0.85, 0.35, M_PI/2, 10000, 2);
}



//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  enablePID = false;
  float backLeftSpeed = 0;
  float frontLeftSpeed = 0;
  float backRightSpeed = 0;
  float frontRightSpeed = 0;
  // task flyWheelTask(flyWheelPIDTask);

  while (true) {
    int joystickAxis3 = Controller1.Axis3.value();
    int joystickAxis4 = Controller1.Axis4.value();
    int joystickAxis1 = Controller1.Axis1.value();
  
    joystickAxis3 = abs(joystickAxis3) < 10 ? 0 : joystickAxis3;
    joystickAxis4 = abs(joystickAxis4) < 10 ? 0 : joystickAxis4;
    joystickAxis1 = abs(joystickAxis1) < 10 ? 0 : joystickAxis1;

    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("Field Oriented: %s", fieldOriented ? "on" : "off");
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("Fly Wheel Speed: %f", flyWheelSpeed);
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("----------------------");

    // printf("\n Brightnesss %f", opticalSensor.brightness());
    // printf("\n Hue %f", opticalSensor.hue());
    
    // Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel1.velocity(rpm));
    // Brain.Screen.newLine();
    // Brain.Screen.print("Fly Wheel 2 Speed: %f", flyWheel2.velocity(rpm));
    // Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 3: %d ", joystickAxis3);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 4: %d ", joystickAxis4);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 1: %d ", joystickAxis1);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Front Left: %f", frontLeftSpeed);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Front Right: %f", frontRightSpeed);
    //   Brain.Screen.newLine();    
    //   Brain.Screen.print("Back Left: %f", backLeftSpeed);
    //   Brain.Screen.newLine();    
    //   Brain.Screen.print("Back Right: %f", backRightSpeed);
    //   Brain.Screen.newLine();

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

    //   Brain.Screen.print("Y: %d ", y);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("X: %d ", x);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Turn: %d ", turn);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Inertial Heading: %f", inertialSensor.heading());
    //   Brain.Screen.newLine();
    } else {
      backLeftSpeed = ((joystickAxis3 - joystickAxis4 + joystickAxis1));
      frontLeftSpeed = ((joystickAxis3 + joystickAxis4 + joystickAxis1));
      backRightSpeed = ((joystickAxis3 + joystickAxis4 - joystickAxis1));
      frontRightSpeed = ((joystickAxis3 - joystickAxis4 - joystickAxis1)); 
    //   Brain.Screen.print("Inertial Heading: %f", inertialSensor.heading());
    //   Brain.Screen.newLine();
    }

    backLeft.spin(vex::directionType::fwd, backLeftSpeed, vex::velocityUnits::pct); //consider doing voltage
    frontLeft.spin(vex::directionType::fwd, frontLeftSpeed, vex::velocityUnits::pct);
    backRight.spin(vex::directionType::fwd, backRightSpeed, vex::velocityUnits::pct);
    frontRight.spin(vex::directionType::fwd, frontRightSpeed, vex::velocityUnits::pct);

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
  Controller1.ButtonUp.pressed(increaseFlyWheelSpeed);
  Controller1.ButtonDown.pressed(decreaseFlyWheelSpeed);
  Controller1.ButtonY.pressed(rollerBlue);
  Controller1.ButtonL1.pressed(toggleIntake);
  Controller1.ButtonA.pressed(toggleFlyWheel);
  Controller1.ButtonLeft.pressed(expand);
  Controller1.ButtonB.pressed(controlSwitch);


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