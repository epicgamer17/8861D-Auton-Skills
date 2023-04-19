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
// backLeft             motor         12              
// frontLeft            motor         11              
// backRight            motor         19              
// frontRight           motor         20              
// intake               motor         1               
// indexer              motor         14              
// expansion            digital_out   A               
// Controller1          controller                    
// inertialSensor       inertial      15              
// sidewaysRotation     rotation      9               
// forwardRotation      rotation      8               
// opticalSensor        optical       18              
// visionSensor         vision        16              
// Controller2          controller                    
// flyWheel1d           motor         6               
// flyWheel2d           motor         7               
// distanceSensorBack   distance      10              
// distanceSensorRight  distance      17              
// distanceSensorLeft   distance      13              
// flyWheel             motor_group   2, 3            
// discCounter          distance      4               
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

float goalPosX = 3.2;
float goalPosY = 0.45;

// #pragma endregion Odometry
#pragma endregion Variables

float ptToPtDistance (float x1, float y1, float x2, float y2) {
  float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
  return dist;
}

void setFlyWheelSpeed(float x, float y) {
  if (ptToPtDistance(x, y, 0.47, 3.2) < ptToPtDistance(x, y, 3.2, 0.47)) {
    flyWheelSpeed = 105 * ptToPtDistance(globalX, globalY, 0.45, 3.2) + 307;
  } else {
    flyWheelSpeed = 105 * ptToPtDistance(globalX, globalY, 3.2, 0.47) + 307;
  }
  if (flyWheelSpeed > 580) {
    flyWheelSpeed = 580;
  }
}

void aimToHighGoal() {
  if (ptToPtDistance(globalX, globalY, 0.47, 3.2) < ptToPtDistance(globalX, globalY, 3.2, 0.47)) {
    turnToPoint(0.47, 3.2, 3500, false, false);
    flyWheelSpeed = 109 * ptToPtDistance(globalX, globalY, 0.47, 3.) + 307;
  } else {
    turnToPoint(3.2, 0.47, 3500, false, false);
    flyWheelSpeed = 109 * ptToPtDistance(globalX, globalY, 3.2, 0.47) + 307; 
  }
  waitUntil(enablePID == false);
}

void shootInHighGoal(int numShots = 1) {
  for (int i = 0; i < 1; i ++) {
    // countDiscs();
    // while (discCount < 1) {
    //   countDiscs();
    //   wait(50, msec);
    // }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
}

void goToFunnel() {
  flyWheelSpeed = 467;
  toggleFlyWheel();
  if (ptToPtDistance(globalX, globalY, 1.76, 0) < ptToPtDistance(globalX, globalY, 1.83, 3.65)) {
  driveTo(1.76, 0.40, 0, 1000, 1, false);
  waitUntil(enablePID == false);
  correctPosition(0);
  waitUntil(enablePID == false);
  driveTo(1.76, 0.20, 0, 750, 1, false);
  turnToPoint(3.2, 0.47, 1000, false, true);
  waitUntil(enablePID == false);
  } else {
    driveTo(1.86, 3.25, M_PI, 1000, 1, false);
    waitUntil(enablePID == false);
    correctPosition(180);
    waitUntil(enablePID == false);
    driveTo(1.86, 3.45, 0, 750, 1, false);
    turnToPoint(0.47, 3.2, 1000, false, true); 
    waitUntil(enablePID == false);
  }
  toggleIntake();
}

void correctPositionNearest() {
  correctingPosition = true;
  if ((round(currentAbsoluteOrientation * 2 / M_PI) == 0)) {
    turnTo(0, 1000);
    waitUntil(enablePID == false);
    correctPosition(0);
  } else if ((round(currentAbsoluteOrientation * 2 / M_PI) == 1)) {
    turnTo(M_PI/2, 1000);
    waitUntil(enablePID == false);
    correctPosition(90);
  } else if ((round(currentAbsoluteOrientation * 2 / M_PI) == 2)) {
    turnTo(M_PI, 1000);
    waitUntil(enablePID == false);
    correctPosition(180);
  } else if ((round(currentAbsoluteOrientation * 2 / M_PI) == 3)) {
    turnTo(3*M_PI/2, 1000);
    waitUntil(enablePID == false);
    correctPosition(270);
  }
}

void goToNearestRoller() {
  if (ptToPtDistance(globalX, globalY, 0.525, 0.525) < ptToPtDistance(globalX, globalY, 3.075, 3.075)) {  
    if (ptToPtDistance(globalX, globalY, 0.15, 0.735) < ptToPtDistance(globalX, globalY, 0.735, 0.15)) {
      driveTo(0.6, 0.75, 0, 2000, 1, false);
      waitUntil(enablePID == false);
      correctPosition(0);
      driveTo(0, 0.75, 0, 1000, 1, true);
      waitUntil(enablePID == false);
    } else {
      driveTo(0.77, 0.6, M_PI/2, 2000, 1, false);
      waitUntil(enablePID == false);
      correctPosition(90);
      driveTo(0.77, 0, M_PI/2, 1000, 1, true);
      waitUntil(enablePID == false);
    }
  } else {
    if (ptToPtDistance(globalX, globalY, 2.9, 3.45) < ptToPtDistance(globalX, globalY, 3.45, 2.9)) {
      driveTo(2.9, 3.05, 3*M_PI/2, 2000, 1, false);
      waitUntil(enablePID == false);
      correctPosition(270);
      driveTo(2.9, 3.65, M_PI/2, 1000, 1, true);
      waitUntil(enablePID == false);

    } else {
      driveTo(3.05, 2.9, M_PI, 2000, 1, false);
      waitUntil(enablePID == false);
      correctPosition(180);
      driveTo(3.65, 2.9, M_PI, 1000, 1, true);
      waitUntil(enablePID == false);

    }
  }
}

void aimAndShootInHighGoal() {
  aimToHighGoal();
  toggleFlyWheel();
  shootInHighGoal(1);
  toggleFlyWheel();
}

void spinNearestRoller() {
  goToNearestRoller();
  rollerRed();
}

void expand() {
  expansion.set(true);
}

void expandButton() {
  if (Controller2.ButtonA.pressing()) {
    expand();
  }
} 

int autonExpand() {
  wait (59, sec);
  expand();
  return 0;
}

void toggleFieldOriented() {
  fieldOriented = !fieldOriented;
  inertialSensor.resetHeading();
}

//Pre Autonomous
void pre_auton() {
  expansion.set(false);
  vexcodeInit();
  
  // inertialSensor.startCalibration();
  sidewaysRotation.resetPosition();
  forwardRotation.resetPosition();

  indexer.setBrake(brakeType::coast);
  indexer.setStopping(brakeType::coast);
  indexer.setTimeout(1, timeUnits::sec);
  // flyWheel1.setStopping(brakeType::coast);
  // flyWheel2.setStopping(brakeType::coast);
  // flyWheel1.setBrake(brakeType::coast);
  // flyWheel2.setBrake(brakeType::coast);

  frontLeft.setStopping(brakeType::coast);
  backLeft.setStopping(brakeType::coast);
  frontRight.setStopping(brakeType::coast);
  backRight.setStopping(brakeType::coast); 


  inertialSensor.calibrate();

  while(inertialSensor.isCalibrating()) {
    task::sleep(100);
  }
  ////DONT FORGET TO SET THE INERTIAL HEADING TO THE START HEADING 
  // inertialSensor.setRotation(-90, deg);
  // inertialSensor.setHeading(-90, deg);
  inertialSensor.setRotation(0, deg);
  inertialSensor.setHeading(0, deg);



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
  timer funnelTimeout = timer();
  
  ////////// AUTON SKILLS//////////
  //Start At Funnel? Shoot 7 Discs//
  // turnTo(M_PI/2, 1000); 
  // flyWheelSpeed = 467;
  setFlyWheelSpeed(1.76, 0.20);
  toggleFlyWheel();
  driveTo(1.76, 0.20, currentAbsoluteOrientation, 2000, 1, false);
  turnToPoint(3.2, 0.45, 2000, false, true);
  waitUntil(enablePID == false);
  for (int i = 0; i < 2; i ++) {
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleIntake();
  
  funnelTimeout.reset();
  for (int i = 0; i < 7; i ++) {
    countDiscs();
    while (discCount < 1) {
      if (funnelTimeout.time(msec) > 15000) {
        break;
      }
      countDiscs();
      wait(50, msec);
    }
    if (funnelTimeout.time(msec) > 15000) {
      break;
    }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();
  toggleIntake();

  //First Roller// 
  driveTo(0.72, 0.5, M_PI/2, 2000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  // correctPosition(90); // fixing odometry 
  correctPositionNearest(); 
  driveTo(0.72, 0, M_PI/2, 1250, 1, true);
    waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);
  
  //Pick Up Disc at 0.6 0.6//
  driveTo(1, 0.6, 0, 1000, 1, false); // could try and get rid of this... not sure though... 
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(0.6, 0.6, 0, 750, 1, false);
  waitUntil(enablePID == false);

  //Second Roller//
  driveTo(0.4, 0.8, 0, 1000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  // correctPosition(0);
  correctPositionNearest();  
  toggleIntake();
  driveTo(0, 0.8, 0, 1250, 1, true);
    waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);

  //Shoot in High Goal//
  // flyWheelSpeed = 515;
  setFlyWheelSpeed(0.6, 1.2);
  toggleFlyWheel();
  driveTo(0.6, 1.2, M_PI/2, 1000, 1, false);
  waitUntil(enablePID == false);
  turnToPoint(0.45, 3.2, 1500, false, false);
  waitUntil(enablePID == false);
  // shootInHighGoal(1);
  countDiscs();
  for (int i = 0; i < 2; i ++) { //try i < discCount
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();

  //Get More Discs While Going Towards Other Corner// 
  turnTo(5*M_PI/4, 1500);
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(1.5, 2.1, 5*M_PI/4, 2500, 0.75, false);
  turnToPoint(1.5, 2.1, 2500, true, true);
  waitUntil(enablePID == false);
  
  //Second Funnel//
  // flyWheelSpeed = 467;
  setFlyWheelSpeed(1.88, 3.3);
  toggleFlyWheel();
  
  driveTo(1.88, 3.3, M_PI, 2500, 1, false);
  waitUntil(enablePID == false);

  // correctPosition(180);
  correctPositionNearest(); 
  driveTo(1.88, 3.45, currentAbsoluteOrientation, 1500, 1, false);
  turnToPoint(0.45, 3.2, 1500, false, true);
  waitUntil(enablePID == false);
  
  // turnToPoint(0.45, 3.2, 2000, false, false); //commenting this out but it might help with aim
  waitUntil(enablePID == false);
  countDiscs();
  for (int i = 0; i < 3; i ++) { //try i < discCount
    countDiscs();
    while (discCount < 1) {
      countDiscs();
      wait(50, msec);
    }
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }

  funnelTimeout.reset();
  for (int i = 0; i < 7; i ++) {
    countDiscs();
    while (discCount < 1) {
      if (funnelTimeout.time(msec) > 15000) {
        break;
      }
      countDiscs();
      wait(100, msec);
    }
    if (funnelTimeout.time(msec) > 15000) {
      break;
    }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();
  toggleIntake();

  //Third Roller//
  driveTo(2.95, 3.15, 3*M_PI/2, 2000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  // correctPosition(270); // fixing odometry 
  correctPositionNearest(); 
  toggleIntake();
  driveTo(2.95, 3.65, 3*M_PI/2, 1750, 1, true);
    waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);

  //Fourth Roller//
  driveTo(3.25, 2.85, M_PI, 1250, 1, false); //0.31 before
  waitUntil(enablePID == false);
  // correctPosition(180); // fixing odometry 
  correctPositionNearest(); 
  toggleIntake();
  driveTo(3.65, 2.85, M_PI, 1750, 1, true);
  waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);

  //Expansion (Should Already Be Automatic)//
  driveTo(3.01, 3.01, 5*M_PI/4, 1000, 1, false);
  waitUntil(enablePID == false);
  expand();
  toggleIntake();
}

void autonSkillsOptimal() {
  timer funnelTimeout = timer();
  
  ////////// AUTON SKILLS//////////
  //First Roller// 
  // driveTo(0.4, 0.8, 0, 3000, 1, false); //0.31 before
  // waitUntil(enablePID == false);
  // correctPosition(0); 
  driveTo(0, 0.8, 0, 1000, 1, true);
  waitUntil(enablePID == false);
  rollerRed();
  waitUntil(enablePID == false);
  
  //Pick Up Disc at 0.6 0.6//
  driveTo(0.625, 1, M_PI/2, 1000, 1, false); // could try and get rid of this... not sure though... 
  waitUntil(enablePID == false);
  correctPosition(90);
  toggleIntake();
  driveTo(0.625, 0.6, M_PI/2, 750, 1, false);
  waitUntil(enablePID == false);

  //Aim to High Goal 
  // flyWheelSpeed = 558;
  setFlyWheelSpeed(globalX, globalY);
  toggleFlyWheel();
  turnToPoint(0.45, 3.2, 1500, false, false);
  waitUntil(enablePID == false);

  //Shoot in High Goal
  for (int i = 0; i < 3; i ++) { //try i < discCount
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleIntake();
  toggleFlyWheel();

  //Second Roller//
  driveTo(0.72, 0.3, M_PI/2, 1500, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(90); // fixing odometry 
  driveTo(0.72, 0, M_PI/2, 500, 1, true);
    waitUntil(enablePID == false);
  rollerRed();
  waitUntil(enablePID == false);
  driveTo(0.9, 0.5, M_PI/2, 1000, 1, false); //0.31 before
  waitUntil(enablePID == false);

  //Three Stack
  // turnToPoint(0.9, 0.9, 1000, true, false);
  // waitUntil(enablePID == false);
  // driveTo(0.9, 0.9, currentAbsoluteOrientation, 3000, 0.5, false);
  // waitUntil(enablePID == false); 
  turnTo(3*M_PI/2, 2000);
  waitUntil(enablePID == false);
  // correctPosition(270);
  driveTo(0.9, 0.85, 3*M_PI/2, 2000, 1, false);
  waitUntil(enablePID == false);
  toggleIntake();
  // flyWheelSpeed = 530;
  setFlyWheelSpeed(0.9, 1.3);
  toggleFlyWheel();
  driveTo(0.9, 1.3, 3*M_PI/2, 2000, 0.35, false);
  waitUntil(enablePID == false);
  turnToPoint(3.2, 0.45, 3000, false, false); //in theory 3.35, 0.35
  waitUntil(enablePID == false);

  //Shoot in High Goal
  for (int i = 0; i < 3; i ++) { //try i < discCount
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();

  //Another Three Stack
  turnTo(M_PI, 1000);
  waitUntil(enablePID == false);
  correctPosition(180);
  driveTo(1.2, 0.9, M_PI, 750, 1, false);
  waitUntil(enablePID == false);
  driveTo(1.76, 0.9, M_PI, 4500, 0.4, false);
  waitUntil(enablePID == false);

  // First Funnel 
  // flyWheelSpeed = 467;
  setFlyWheelSpeed(1.76, 0.40);
  toggleFlyWheel();
  driveTo(1.76, 0.40, 0, 6000, 1, false);
  waitUntil(enablePID == false);
  correctPosition(0);
  waitUntil(enablePID == false);
  driveTo(1.76, 0.20, currentAbsoluteOrientation, 750, 1, false);
  turnToPoint(3.2, 0.45, 2000, false, true); 
  waitUntil(enablePID == false);
  for (int i = 0; i < 3; i ++) {
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(1000, msec);
  }
  toggleIntake();
  
  funnelTimeout.reset();
  for (int i = 0; i < 7; i ++) {
    countDiscs();
    while (discCount < 1) {
      if (funnelTimeout.time(msec) > 17000) {
        break;
      }
      countDiscs();
      wait(50, msec);
    }
    if (funnelTimeout.time(msec) > 17000) {
      break;
    }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();
  toggleIntake();
  

  //Second Funnel//
  // flyWheelSpeed = 467;
  setFlyWheelSpeed(1.88, 3.3);
  toggleFlyWheel();
  
  driveTo(1.88, 3.3, M_PI, 6000, 1, false);
  waitUntil(enablePID == false);

  correctPosition(180);
  driveTo(1.88, 3.45, currentAbsoluteOrientation, 2000, 1, false);
  turnToPoint(0.45, 3.2, 2000, false, true);
  waitUntil(enablePID == false);
  
  // turnToPoint(0.45, 3.2, 2000, false, false); //commenting this out but it might help with aim
  waitUntil(enablePID == false);
  countDiscs();
  for (int i = 0; i < 3; i ++) { //try i < discCount
    countDiscs();
    while (discCount < 1) {
      countDiscs();
      wait(100, msec);
    }
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }

  funnelTimeout.reset();
  for (int i = 0; i < 7; i ++) {
    countDiscs();
    while (discCount < 1) {
      if (funnelTimeout.time(msec) > 17000) {
        break;
      }
      countDiscs();
      wait(50, msec);
    }
    if (funnelTimeout.time(msec) > 17000) {
      break;
    }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();
  toggleIntake();

  //Three Stack
  driveTo(1.95, 2.73, M_PI, 1000, 1, false);
  waitUntil(enablePID == false);
  correctPosition(180);
  toggleIntake();
  // flyWheelSpeed = 550;
  setFlyWheelSpeed(2.13, 2.73);
  toggleFlyWheel();
  driveTo(2.13, 2.73, 0, 3000, 1, false);
  waitUntil(enablePID == false);
  turnToPoint(3.2, 0.45, 2000, false, true);
  waitUntil(enablePID == false);

  //Shoot in High Goal
  for (int i = 0; i < 3; i ++) { //try i < discCount
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }

  //Another Three Stack 
  // flyWheelSpeed = 550;
  setFlyWheelSpeed(2.73, 2.73);
  driveTo(2.73, 2.73, 0, 3000, 1, false);
  waitUntil(enablePID == false);
  turnToPoint(3.2, 0.45, 2000, false, true); //3.3, 0.35 //3.15, 0.35
  waitUntil(enablePID == false);

  //Shoot in High Goal
  for (int i = 0; i < 3; i ++) { //try i < discCount
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }


  //Third Roller//
  driveTo(2.95, 3.25, 3*M_PI/2, 2000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(270); // fixing odometry 
  toggleIntake();
  driveTo(2.95, 3.65, 3*M_PI/2, 1750, 1, true);
  rollerRed();
  waitUntil(enablePID == false);

  //Fourth Roller//
  driveTo(3.25, 2.85, M_PI, 1250, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(180); // fixing odometry 
  toggleIntake();
  driveTo(3.65, 2.85, M_PI, 2000, 1, true);
  rollerRed();
  waitUntil(enablePID == false);

  //Expansion (Should Already Be Automatic)//
  driveTo(3.01, 3.01, 5*M_PI/4, 1000, 1, false);
  waitUntil(enablePID == false);
  expand();
}

void autonSkillsV2() {
  timer funnelTimeout = timer();
  
  ////////// AUTON SKILLS//////////
  //Start At Funnel? Shoot 7 Discs//
  // turnTo(M_PI/2, 1000); 
  flyWheelSpeed = 467;
  // setFlyWheelSpeed(1.76, 0.20);
  toggleFlyWheel();
  driveTo(1.76, 0.20, currentAbsoluteOrientation, 2000, 1, false);
  turnToPoint(3.2, 0.47, 2500, false, true);
  waitUntil(enablePID == false);
  for (int i = 0; i < 2; i ++) {
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleIntake();
  
  funnelTimeout.reset();
  for (int i = 0; i < 7; i ++) {
    countDiscs();
    while (discCount < 1) {
      if (funnelTimeout.time(msec) > 14000) {
        break;
      }
      countDiscs();
      wait(50, msec);
    }
    if (funnelTimeout.time(msec) > 14000) {
      break;
    }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();
  toggleIntake();

  //First Roller// 
  driveTo(0.76, 0.5, M_PI/2, 2000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(90); // fixing odometry 
  // correctPositionNearest(); 
  driveTo(0.76, 0, M_PI/2, 1250, 1, true);
  waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);
  
  //Pick Up Disc at 0.6 0.6//
  driveTo(1, 0.6, 0, 1000, 1, false); // could try and get rid of this... not sure though... 
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(0.6, 0.6, 0, 750, 1, false);
  waitUntil(enablePID == false);

  //Second Roller//
  driveTo(0.4, 0.75, 0, 1000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(0);
  // correctPositionNearest();  
  toggleIntake();
  driveTo(0, 0.75, 0, 1250, 1, true);
  waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);

  //Shoot in High Goal//
  flyWheelSpeed = 515;
  // setFlyWheelSpeed(0.6, 1.2);
  toggleFlyWheel();
  driveTo(0.6, 1.2, M_PI/2, 1000, 1, false);
  waitUntil(enablePID == false);
  turnToPoint(0.45, 3.2, 1500, false, false);
  waitUntil(enablePID == false);
  // shootInHighGoal(1);
  countDiscs();
  for (int i = 0; i < 2; i ++) { //try i < discCount
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();

  //Get More Discs While Going Towards Other Corner// 
  turnTo(5*M_PI/4, 1500);
  waitUntil(enablePID == false);
  toggleIntake();
  driveTo(1.5, 2.1, 5*M_PI/4, 2500, 0.75, false);
  turnToPoint(1.5, 2.1, 2500, true, true);
  waitUntil(enablePID == false);
  
  flyWheelSpeed = 467;
  // setFlyWheelSpeed(1.88, 3.3);
  toggleFlyWheel();
  
  driveTo(1.86, 3.3, M_PI, 2500, 1, false);
  waitUntil(enablePID == false);

  correctPosition(180);
  correctPositionNearest(); 
  driveTo(1.86, 3.45, currentAbsoluteOrientation, 1500, 1, false);
  turnToPoint(0.45, 3.17, 2500, false, true);
  waitUntil(enablePID == false);
  
  // turnToPoint(0.45, 3.2, 2000, false, false); //commenting this out but it might help with aim
  waitUntil(enablePID == false);
  countDiscs();
  for (int i = 0; i < 3; i ++) { //try i < discCount
    countDiscs();
    while (discCount < 1) {
      countDiscs();
      wait(50, msec);
    }
    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }

  funnelTimeout.reset();
  for (int i = 0; i < 7; i ++) {
    countDiscs();
    while (discCount < 1) {
      if (funnelTimeout.time(msec) > 14000) {
        break;
      }
      countDiscs();
      wait(100, msec);
    }
    if (funnelTimeout.time(msec) > 14000) {
      break;
    }

    waitUntil(fabs(flyWheelError) < 10); //flyWheelSpeed/50
    index();
    wait(500, msec);
  }
  toggleFlyWheel();
  toggleIntake();

  //Third Roller//
  driveTo(2.90, 3.15, 3*M_PI/2, 2000, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(270); // fixing odometry 
  // correctPositionNearest(); 
  toggleIntake();
  driveTo(2.90, 3.65, 3*M_PI/2, 1750, 1, true);
  waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);

  //Fourth Roller//
  driveTo(3.25, 2.9, M_PI, 1250, 1, false); //0.31 before
  waitUntil(enablePID == false);
  correctPosition(180); // fixing odometry 
  // correctPositionNearest(); 
  toggleIntake();
  driveTo(3.65, 2.9, M_PI, 1750, 1, true);
  waitUntil(enablePID == false); //comment if rollers break
  rollerRed();
  waitUntil(enablePID == false);

  //Expansion (Should Already Be Automatic)//
  driveTo(3.01, 3.01, 5*M_PI/4, 1000, 1, false);
  waitUntil(enablePID == false);
  expand();
  toggleIntake();
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
  
  indexer.setBrake(brakeType::coast);
  indexer.setStopping(brakeType::coast);
  indexer.setTimeout(1, timeUnits::sec);

  //start the odometry
  task odometryTask(positionTracking);
  // task IMUodometry(IMUPositionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  task failSafeExpansion(autonExpand);
  task flyWheelTask(flyWheelPICTask);
  // visionPickUpDisc();

  expansion.set(false);
  
  correctPosition(0); //set start position
  // autonSkillsV1();
  autonSkillsV2();
  // competitionAutonomous();
}

//User Control
void usercontrol() { // Try also applying PID, also maybe PID on the flywheel?? 
  //reset rotation sensors
  forwardRotation.resetPosition();
  sidewaysRotation.resetPosition();

  frontLeft.resetRotation();
  frontRight.resetRotation();
  backLeft.resetRotation();
  backRight.resetRotation();
  
  indexer.setBrake(brakeType::coast);
  indexer.setStopping(brakeType::coast);
  indexer.setTimeout(1, timeUnits::sec);

  //start the odometry
  task odometryTask(positionTracking);
  // task IMUodometry(IMUPositionTracking);
  task drawFieldTask(drawField); // uncomment after testing turning
  task pidTask(PIDTask);
  // task purePursuit(PPSTask);
  // task intakeTask(intakeControl);
  task failSafeExpansion(autonExpand);
  task flyWheelTask(flyWheelPICTask);
  // visionPickUpDisc();

  expansion.set(false);
  
  correctPosition(0); //set start position
  // autonSkillsV1();
  autonSkillsV2();
  // competitionAutonomous();
  // task odometryTask(positionTracking);
  // // task IMUodometry(IMUPositionTracking);
  // task drawFieldTask(drawField); // uncomment after testing turning
  // task pidTask(PIDTask);
  // // task purePursuit(PPSTask);
  // // task intakeTask(intakeControl);
  // task flyWheelTask(flyWheelPICTask);

  // desiredX = globalX;
  // desiredY = globalY;
  // enablePID = false;
  // userControl = true;

  // float backLeftSpeed = 0;
  // float frontLeftSpeed = 0;
  // float backRightSpeed = 0;
  // float frontRightSpeed = 0;

  // indexer.setBrake(brakeType::coast);
  // indexer.setStopping(brakeType::coast);
  // indexer.setTimeout(1, timeUnits::sec);

  // correctPosition(0); //set start position

  // // driveTo(1.76, 0.20, currentAbsoluteOrientation, 1000, 1, false);
  // // turnToPoint(3.2, 0.45, 2000, false, true);
  // waitUntil(enablePID == false);

  // while (true) {
  //   int joystickAxis3 = Controller1.Axis3.value();
  //   int joystickAxis4 = Controller1.Axis4.value();
  //   int joystickAxis1 = Controller1.Axis1.value()  * 0.7;
  
  //   joystickAxis3 = abs(joystickAxis3) < 15 ? 0 : joystickAxis3;
  //   joystickAxis4 = abs(joystickAxis4) < 15 ? 0 : joystickAxis4;
  //   joystickAxis1 = abs(joystickAxis1) < 15 ? 0 : joystickAxis1;


  //   // printf("\n Brightnesss %f", opticalSensor.brightness());
  //   // printf("\n Hue %f", opticalSensor.hue());


  //   if (fieldOriented == true) {
  //     int turn = joystickAxis1;
  //     int x = joystickAxis4;
  //     int y = joystickAxis3;
  //     float inertialRadians = (inertialSensor.heading() - 45) * (M_PI/180); // + M_PI/2 Because odom has orientation as M_PI/2
  //     float temp = y * cos(inertialRadians) + x * sin(inertialRadians);
  //     x = - y * sin(inertialRadians) + x * cos(inertialRadians);
  //     y = temp;

  //     backLeftSpeed = (y + turn);
  //     frontLeftSpeed = (x + turn);
  //     backRightSpeed = (x - turn);
  //     frontRightSpeed = (y - turn);
  //   } else {
  //     backLeftSpeed = ((joystickAxis3 - joystickAxis4 + joystickAxis1));
  //     frontLeftSpeed = ((joystickAxis3 + joystickAxis4 + joystickAxis1));
  //     backRightSpeed = ((joystickAxis3 + joystickAxis4 - joystickAxis1));
  //     frontRightSpeed = ((joystickAxis3 - joystickAxis4 - joystickAxis1)); 
  //   }

  //   if (joystickAxis1 != 0 || joystickAxis3 != 0 || joystickAxis4 != 0) {
  //     // enablePID = true;
  //     // desiredX = globalX + (joystickAxis4 * sin(currentAbsoluteOrientation)) + (joystickAxis3 * cos(currentAbsoluteOrientation));
  //     // desiredY = globalY + (joystickAxis4 * sin(currentAbsoluteOrientation)) + (joystickAxis3 * cos(currentAbsoluteOrientation));
  //     // desiredHeading = currentAbsoluteOrientation + (joystickAxis1/(100*M_PI));

  //     // desiredX = globalX + (joystickAxis4/100);
  //     // desiredY = globalY + (joystickAxis3/100);
  //     // desiredHeading = currentAbsoluteOrientation + (joystickAxis1/(100*M_PI));


  //     enablePID = false;
  //     desiredX = globalX;
  //     desiredY = globalY;
  //     desiredHeading = currentAbsoluteOrientation;
  //     backLeft.spin(vex::directionType::fwd, backLeftSpeed, vex::velocityUnits::pct); //consider doing voltage
  //     frontLeft.spin(vex::directionType::fwd, frontLeftSpeed, vex::velocityUnits::pct);
  //     backRight.spin(vex::directionType::fwd, backRightSpeed, vex::velocityUnits::pct);
  //     frontRight.spin(vex::directionType::fwd, frontRightSpeed, vex::velocityUnits::pct);
  //   } else if (enablePID == false) {
  //     backLeft.stop(coast); //consider doing voltage
  //     frontLeft.stop(coast);
  //     backRight.stop(coast);
  //     frontRight.stop(coast);
  //     desiredX = globalX;
  //     desiredY = globalY;
  //     desiredHeading = currentAbsoluteOrientation;
  //   }
  //   if (Controller1.ButtonL2.pressing()) 
  //   {
  //     intake.spin(reverse, 40, velocityUnits::pct);    
  //     intakeOn = false;
  //   }
  //   else 
  //   {
  //     if (intakeOn == false)
  //     {
  //       intake.stop();
  //     }
  //   }
  //   //Don't hog the CPU
  //   // wait(20, msec);
  // }
}

int main() {
  // Event Registration for Buttons
  Controller1.ButtonR1.pressed(index);
  Controller1.ButtonL1.pressed(toggleIntake);
  Controller1.ButtonR2.pressed(aimToHighGoal);
  //L2 Used Above in User Control
  Controller1.ButtonUp.pressed(increaseFlyWheelSpeed);
  Controller1.ButtonDown.pressed(decreaseFlyWheelSpeed);
  // Controller1.ButtonLeft.pressed();
  Controller1.ButtonRight.pressed(goToFunnel);
  Controller1.ButtonA.pressed(toggleFlyWheel);
  Controller1.ButtonB.pressed(toggleFieldOriented);
  Controller1.ButtonY.pressed(rollerRed);
  Controller1.ButtonX.pressed(goToNearestRoller);

  //add shoot in highgoal/set flywheel speed
  //add switch roller color from red to blue / blue to red.
  Controller2.ButtonR1.pressed(correctPositionNearest);
  // Controller2.ButtonL1.pressed();
  Controller2.ButtonR2.pressed(aimAndShootInHighGoal);
  Controller2.ButtonL2.pressed(spinNearestRoller);
  // Controller2.ButtonUp.pressed();
  // Controller2.ButtonDown.pressed();
  Controller2.ButtonLeft.pressed(expandButton);
  // Controller2.ButtonRight.pressed();
  //Button A used for Expansion With Button Left
  // Controller2.ButtonB.pressed();
  // Controller2.ButtonY.pressed();
  // Controller2.ButtonX.pressed();

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