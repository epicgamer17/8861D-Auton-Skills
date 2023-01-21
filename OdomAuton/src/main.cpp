#pragma region VEXcode Generated Robot Configuration

#include "vex.h"
#include "PID.h"
#include "draw-field.h"
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
// ---- END VEXCODE CONFIGURED DEVICES ----
// Robot configuration code.
#pragma endregion VEXcode Generated Robot Configuration

using namespace vex;

competition Competition; 

#pragma region Variables
// Variables 
int flyWheelSpeed = 10; // 80 for pct, i am using 10 for voltage 
float j = 0.0125/360; //Turning correction constant i think im not sure ask friend  
float tileLength = 0.4572; //  0.6096
float wheelRadius = 0.041275;

bool stopIntake = true;
bool intakeOn = false;
bool flyWheelOn = false;
bool fieldOriented = false;

// #pragma endregion Odometry
#pragma endregion Variables

//Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

//Index Function
void index() {
  indexer.startSpinFor(1, rev, 100, velocityUnits::pct);
}

void increaseFlyWheelSpeed() {
  if (flyWheelSpeed < 12) {  // < 100 for pct
    flyWheelSpeed += 1;
  }
}

void decreaseFlyWheelSpeed() {
  if (flyWheelSpeed > 0) {
    flyWheelSpeed -= 1;
  }
}

void rollerSingleRotation() {
  intake.spinFor(fwd, 1, rev, 100, velocityUnits::pct, true);
}

void toggleIntake() {
  if (intakeOn == false) {
    intake.spin(directionType::rev, 100, velocityUnits::pct);
    intakeOn = true;
  } else if (intakeOn == true) {
    intake.stop();
    intakeOn = false;
  }
}

void toggleFlyWheel() {
  if (flyWheelOn == false) {
    flyWheel1.spin(directionType::fwd, -flyWheelSpeed * 50, velocityUnits::rpm);
    flyWheel2.spin(directionType::fwd, -flyWheelSpeed * 50, velocityUnits::rpm);
    // flyWheel1.spin(directionType::fwd, -flyWheelSpeed, voltageUnits::volt);
    // flyWheel2.spin(directionType::fwd, -flyWheelSpeed, voltageUnits::volt);
    flyWheelOn = true;
  } else if (flyWheelOn == true) {
    flyWheel1.stop();
    flyWheel2.stop();
    flyWheelOn = false;
  }
}

void turnroller() {
  // PI/DFunc(-0.05, 0, 0);
  intake.spinFor(fwd, 1, rev, 100, velocityUnits::pct, true);
}

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
  
  inertialSensor.startCalibration();
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
  task drawFieldTask(drawField);
  task pidTask(PIDTask);

  turnTo(M_PI/2, 2500);
  waitUntil(enablePID == false);
  driveTo(0, 0.6, 0, 5000, 2);
  waitUntil(enablePID == false);
  driveTo(0.6, 0.6, 0, 5000, 2);
  waitUntil(enablePID == false);
  driveTo(0, 0, 0, 5000, 2);
  // task intakeTask(intakeControl);
  
  // task failSafeExpansion = vex::task(autonExpand);

  // resetEncoders = true;
  // PIDFunc(0, 0, 0); //desiredValue = dV, desiredTurnValue = dTV, movementDirection = mD;
  // PIDFunc(-0.05, 0, 0);
  // intake.spinFor(fwd, 1, rev, 100, velocityUnits::pct, true);
  // PIDFunc(0.2286, 0, 0);
  // PIDFunc(0, 90, 0);
  // intake.spin(fwd, 100, velocityUnits::pct);
  // PIDFunc(-0.25146, 0, 0);
  // intake.stop();
  // PIDFunc(-0.09144, 0, 0);
  // intake.spinFor(fwd, 1, rev, 100, velocityUnits::pct, true);
  // PIDFunc(0.88298, 0, 0);
  // PIDFunc(0.3, 0, 90);
  // PIDFunc(0.3, 0, 0);
  // PIDFunc(0, -10, 0);
  // PIDFunc(0,0,0);
  // enablePID = false;
  // // flyWheel1.spin(directionType::fwd, -12, voltageUnits::volt);
  // // flyWheel2.spin(directionType::fwd, -12, voltageUnits::volt);
  // flyWheel1.spin(directionType::fwd, -100, velocityUnits::pct);
  // flyWheel2.spin(directionType::fwd, -100, velocityUnits::pct);

  // for (int i = 0; i < 3; i ++) {
  //   timer flyWheelTimeout = timer();
  //   while ((flyWheel1.velocity(rpm) + flyWheel1.velocity(rpm)/2) < 597 && flyWheelTimeout.time() < 3000) {
  //     Brain.Screen.clearScreen();
  //     Brain.Screen.setCursor(1, 1);
  //     Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel1.velocity(rpm));
  //     Brain.Screen.newLine();
  //     Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel2.velocity(rpm));
  //     Brain.Screen.newLine();
  //     Brain.Screen.print("Fly Wheel Timeout: %d", flyWheelTimeout.time());
  //     Brain.Screen.newLine();
  //    wait(10, msec);
  //   }
  //   index();
  // }
  // wait(0.5, sec);
  // flyWheel1.stop();
  // flyWheel2.stop();
  // enablePID = true;
  // PIDFunc(0, 0, 0);
  // PIDFunc(0, 10, 0);
  // PIDFunc(-0.3, 0, 0);
  // PIDFunc(-0.3, 0, 90);
  // PIDFunc(0, 90, 0);
  
  // // PIDFunc(0, 20, 0);
  // // PIDFunc(0,0,0);
  // // enablePID = false;
  // // flyWheel1.spin(directionType::fwd, -12, voltageUnits::volt);
  // // flyWheel2.spin(directionType::fwd, -12, voltageUnits::volt);
  // // wait(1, sec);
  // // index();
  // // wait(1,sec);
  // // index();
  // // wait(1,sec);
  // // index();
  // // wait(1, sec);
  // // flyWheel1.stop();
  // // flyWheel2.stop();
  // // enablePID = true;
  // // PIDFunc(0, 0, 0);
  // // PIDFunc(0, 70, 0);

  // intake.spin(fwd, 100, velocityUnits::pct);
  // PIDFunc(-0.9144, 0, 0);
  // intake.stop();
  // PIDFunc(-0.2286, 0, 0);
  // PIDFunc(0, 90, 0);
  // intake.spin(fwd, 100, velocityUnits::pct);
  // PIDFunc(-0.6858, 0, 0);
  // intake.stop();
  // PIDFunc(-0.1143, 0, 0);
  // intake.spinFor(fwd, 1, rev, 100, velocityUnits::pct, true);
  // PIDFunc(0.2286, 0, 0);
  // PIDFunc(0, -90, 0);
  // PIDFunc(-0.6858, 0, 0);
  // intake.spinFor(fwd, 1, rev, 100, velocityUnits::pct, true);
  // PIDFunc(-0.2286, 0, 0);
  // PIDFunc(0, 45, 0);
  // expand();
}



//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  enablePID = false;
  float backLeftSpeed = 0;
  float frontLeftSpeed = 0;
  float backRightSpeed = 0;
  float frontRightSpeed = 0;
  

  while (true) {
    int joystickAxis3 = Controller1.Axis3.value();
    int joystickAxis4 = Controller1.Axis4.value();
    int joystickAxis1 = Controller1.Axis1.value();
    
    joystickAxis3 = abs(joystickAxis3) < 15 ? 0 : joystickAxis3;
    joystickAxis4 = abs(joystickAxis4) < 15 ? 0 : joystickAxis4;
    joystickAxis1 = abs(joystickAxis1) < 15 ? 0 : joystickAxis1;

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Field Oriented: %s", fieldOriented ? "true" : "false");
    Controller1.Screen.newLine();
    Controller1.Screen.print("Fly Wheel Speed: %d", flyWheelSpeed);
    Controller1.Screen.newLine();
    Controller1.Screen.print("----------------------");


    if (fieldOriented == true) {
      int turn = joystickAxis1;
      int x = joystickAxis4;
      int y = joystickAxis3;
      float inertialRadians = (inertialSensor.heading() - 45) * (M_PI/180);
      float temp = y * cos(inertialRadians) + x * sin(inertialRadians);
      x = - y * sin(inertialRadians) + x * cos(inertialRadians);
      y = temp;

      backLeftSpeed = (y + turn);
      frontLeftSpeed = (x + turn);
      backRightSpeed = (x - turn);
      frontRightSpeed = (y - turn);

    //   Brain.Screen.clearScreen();
    //   Brain.Screen.setCursor(1, 1);
    //   Brain.Screen.print("Front Left: %f", frontLeftSpeed);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Front Right: %f", frontRightSpeed);
    //   Brain.Screen.newLine();    
    //   Brain.Screen.print("Back Left: %f", backLeftSpeed);
    //   Brain.Screen.newLine();    
    //   Brain.Screen.print("Back Right: %f", backRightSpeed);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel1.velocity(rpm));
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel2.velocity(rpm));
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Y: %d ", y);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("X: %d ", x);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Turn: %d ", turn);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 3: %d ", joystickAxis3);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 4: %d ", joystickAxis4);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 1: %d ", joystickAxis1);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Inertial Heading: %f", inertialSensor.heading());
    //   Brain.Screen.newLine();
    // } else {
    //   backLeftSpeed = ((joystickAxis3 - joystickAxis4 + joystickAxis1));
    //   frontLeftSpeed = ((joystickAxis3 + joystickAxis4 + joystickAxis1));
    //   backRightSpeed = ((joystickAxis3 + joystickAxis4 - joystickAxis1));
    //   frontRightSpeed = ((joystickAxis3 - joystickAxis4 - joystickAxis1));
    //   Brain.Screen.clearScreen();
    //   Brain.Screen.setCursor(1, 1);
    //   Brain.Screen.print("Front Left: %f", frontLeftSpeed);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Front Right: %f", frontRightSpeed);
    //   Brain.Screen.newLine();    
    //   Brain.Screen.print("Back Left: %f", backLeftSpeed);
    //   Brain.Screen.newLine();    
    //   Brain.Screen.print("Back Right: %f", backRightSpeed);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel1.velocity(rpm));
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel2.velocity(rpm));
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 3: %d ", joystickAxis3);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 4: %d ", joystickAxis4);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Joystick Axis 1: %d ", joystickAxis1);
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("Inertial Heading: %f", inertialSensor.heading());
    //   Brain.Screen.newLine();
    }

    backLeft.spin(vex::directionType::fwd, backLeftSpeed, vex::velocityUnits::pct); //consider doing voltage
    frontLeft.spin(vex::directionType::fwd, frontLeftSpeed, vex::velocityUnits::pct);
    backRight.spin(vex::directionType::fwd, backRightSpeed, vex::velocityUnits::pct);
    frontRight.spin(vex::directionType::fwd, frontRightSpeed, vex::velocityUnits::pct);

    if (Controller1.ButtonL2.pressing()) 
    {
      intake.spin(directionType::fwd, 100, velocityUnits::pct);    
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
  Controller1.ButtonY.pressed(rollerSingleRotation);
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