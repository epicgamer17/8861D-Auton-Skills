#include "intake.h"

bool intakeOn = false;

// void turnroller() {
//   // PI/DFunc(-0.05, 0, 0);
//   intake.spinFor(reverse, 1, rev, 100, velocityUnits::pct, true);
// }

void rollerBlue() {
  timer rollerTimeout = timer();
  while (opticalSensor.color() != color::blue /*&& rollerTimeout.time() < 2500*/) {
    intake.spin(reverse, 50, velocityUnits::pct);
  }
  intake.stop();
}

void toggleIntake() {
  if (intakeOn == false) {
    intake.spin(fwd, 100, velocityUnits::pct);
    intakeOn = true;
  } else if (intakeOn == true) {
    intake.stop();
    intakeOn = false;
  }
}

#pragma region PID Varialbes
double visionTurnError = 0;
double visionTurnPrevError = 0;

double visionTurnMinError = 0.01;

double visionTurnIntegral = 0;
double visionTurnIntegralBound = 0.09; //i think the units are radians so no need to convert to radians 

double visionTurnDerivative = 0;

double visionTurnkP = 13.00;
double visionTurnkI = 1.00;
double visionTurnkD = 10.00;

double visionTurnPowerPID = 0;

double frontLeftPower = 0;
double frontRightPower = 0;
double backLeftPower = 0;
double backRightPower = 0;

double linearVelocity = 7;

int visionMaxTurningSpeed = 1;
#pragma endregion PID Variables

void visionTurnPID() {
  //Error is equal to the difference between the current facing direction and the target direction
  visionTurnError = currentAbsoluteOrientation - desiredHeading;

  if(fabs(visionTurnError) > M_PI) {
    visionTurnError = (visionTurnError/fabs(visionTurnError)) * -1 * fabs(2 * M_PI - visionTurnError);
  }

  //only use integral if close enough to target
  if(fabs(visionTurnError) < visionTurnIntegralBound) {
    visionTurnIntegral += visionTurnError;
  }
  else {
    visionTurnIntegral = 0;
  }

  //reset integral if we pass the target
  if(visionTurnError * visionTurnPrevError < 0) {
    visionTurnIntegral = 0;
  } 

  visionTurnDerivative = visionTurnError - visionTurnPrevError;

  visionTurnPrevError = visionTurnError;

  visionTurnPowerPID = (visionTurnError * visionTurnkP + visionTurnIntegral * visionTurnkI + visionTurnDerivative * visionTurnkD);

  //Limit power output to 12V
  if(visionTurnPowerPID > 12) {
    visionTurnPowerPID = 12;
  }

  if(fabs(visionTurnError) < visionTurnMinError) {
    visionTurnPowerPID = 0;
  }
}
  
/* CHASSIS CONTROL TASK */
int PIDTask() {

  //loop to constantly execute chassis commands
  while(1) {
    
    if(enablePID) {
      //Distances to target on each axis

      turnPID();

      //set power for each motor
      frontLeftPower = ((linearVelocity) + visionTurnPowerPID);
      frontRightPower = ((linearVelocity) - visionTurnPowerPID);
      backLeftPower = ((linearVelocity) + visionTurnPowerPID);
      backRightPower = ((linearVelocity) - visionTurnPowerPID);

      frontLeft.spin(directionType::fwd, frontLeftPower, voltageUnits::volt);
      frontRight.spin(directionType::fwd, frontRightPower, voltageUnits::volt);
      backLeft.spin(directionType::fwd, backLeftPower, voltageUnits::volt);
      backRight.spin(directionType::fwd, backRightPower, voltageUnits::volt);
   
      // if(fabs(turnError) < 0.003) {
      //   enablePID = false;
      //   turningToPoint = false;
      // }

      // if(Brain.timer(timeUnits::msec) > timeoutLength) {
      //   enablePID = false;
      //   turningToPoint = false;
      // }
    }
    //What to do when not using the chassis controls
    else {
      frontLeft.stop(brakeType::coast);
      frontRight.stop(brakeType::coast);
      backLeft.stop(brakeType::coast);
      backRight.stop(brakeType::coast);
    }
    
    task::sleep(20);
  }

  return 1;
}


void visionPickUpDisc() {
  while (true) {
    visionSensor.takeSnapshot(visionSensor__YELLOW_DISC);
    if (visionSensor.objectCount > 0) {
      visionTurnError = 157.5 - visionSensor.largestObject.centerX;
    } else {
      frontLeft.spin(fwd);
      frontRight.spin(reverse);
      backLeft.spin(fwd);
      backRight.spin(reverse);
    }
    task::sleep(40);
  }
}



// void visionPickUpDisc() {
//   while (true) {
//     visionSensor.takeSnapshot(visionSensor__YELLOW_DISC);
//     // visionSensor.takeSnapshot(yellow);
//     // if (visionSensor.objectCount > 0) {
//     //   if (visionSensor.largestObject.centerX > 255) {
//     //     // turn right
//     //     // turnTo(currentAbsoluteOrientation - M_PI/2, 1000);
//     //     //could do drive forward slowly by certain amount and then set turn error to the 
//     //     //amount off it is from the center of the screen. Could calculate the width by
//     //     // measuring how much the camera can see right and left (what angle)
//     //     // maybe could calculate the point doing something similar, converting pixel values
//     //     frontLeft.spin(reverse);
//     //     frontRight.spin(fwd);
//     //     backLeft.spin(reverse);
//     //     backRight.spin(fwd);
//     //     turnTo(currentAbsoluteOrientation + (((visionSensor.largestObject.centerX - 157.5)/157.5) * M_PI/6), 2500);
//     //   } else if (visionSensor.largestObject.centerX < 60) {
//     //     // turn right
//     //     // turnTo(currentAbsoluteOrientation + M_PI/2, 1000);
//     //     frontLeft.spin(fwd);
//     //     frontRight.spin(reverse);
//     //     backLeft.spin(fwd);
//     //     backRight.spin(reverse);
//     //   } else if (visionSensor.largestObject.centerX > 60 && visionSensor.largestObject.centerX < 255) {
//     //     if (visionSensor.largestObject.width < 160) {
//     //       //drive forward
//     //       // driveTo(globalX + ammount, globalY + ammount, currentAbsoluteOrientation, 1000, 1);
//     //       // turnTo(currentAbsoluteOrientation, 1000);
//     //       frontLeft.spin(reverse);
//     //       frontRight.spin(reverse);
//     //       backLeft.spin(reverse);
//     //       backRight.spin(reverse);
//     //     } else {
//     //       frontLeft.stop(coast);
//     //       frontRight.stop(coast);
//     //       backLeft.stop(coast);
//     //       backRight.stop(coast);
//     //     }
//     //   }
//     // } else {
//     //   // turnTo(currentAbsoluteOrientation + M_PI/2, 1000); //make the robot turn, could just not use PID but try it with this
//     //   frontLeft.spin(fwd);
//     //   frontRight.spin(reverse);
//     //   backLeft.spin(fwd);
//     //   backRight.spin(reverse);
//     // }

//     if (visionSensor.objectCount > 0) {
//       // driveTo(globalX - (cos(currentAbsoluteOrientation) * visionSensor.largestObject.centerY * 0.01125), globalY - (sin(currentAbsoluteOrientation) * visionSensor.largestObject.centerY * 0.01125), currentAbsoluteOrientation, 2500, 1);
//       // if (visionSensor.largestObject.centerX > 255 && visionSensor.largestObject.centerX < 60) {
//       //   // turnTo(currentAbsoluteOrientation + (((visionSensor.largestObject.centerX - 157.5)/157.5) * M_PI/6), 2500);
//       //   //try just mkaing a visionSensor PID which uses the visionSesnor pixels directly instead of converting it to meters, do the same for Pure Pursuit. Also like maybe have a somewhat constant drive velocity (like pure pursuit?)
//       // }
//       visionTurnError = 
//     } else {
//       // turnTo(currentAbsoluteOrientation + M_PI, 2500);
//       frontLeft.spin(fwd);
//       frontRight.spin(reverse);
//       backLeft.spin(fwd);
//       backRight.spin(reverse);
//     }
//     task::sleep(40);
//   }
// }