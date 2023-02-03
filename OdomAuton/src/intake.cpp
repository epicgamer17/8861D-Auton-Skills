#include "intake.h"

bool intakeOn = false;

// void turnroller() {
//   // PI/DFunc(-0.05, 0, 0);
//   intake.spinFor(reverse, 1, rev, 100, velocityUnits::pct, true);
// }

void rollerBlue() {
  timer rollerTimeout = timer();
  while (opticalSensor.color() != color::blue /*&& rollerTimeout.time() < 2500*/) {
    intake.spin(reverse);
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

void visionPickUpDisc() {
  while (true) {
    visionSensor.takeSnapshot(visionSensor__YELLOW_DISC);
    // visionSensor.takeSnapshot(yellow);
    // if (visionSensor.objectCount > 0) {
    //   if (visionSensor.largestObject.centerX > 255) {
    //     // turn right
    //     // turnTo(currentAbsoluteOrientation - M_PI/2, 1000);
    //     //could do drive forward slowly by certain amount and then set turn error to the 
    //     //amount off it is from the center of the screen. Could calculate the width by
    //     // measuring how much the camera can see right and left (what angle)
    //     // maybe could calculate the point doing something similar, converting pixel values
    //     frontLeft.spin(reverse);
    //     frontRight.spin(fwd);
    //     backLeft.spin(reverse);
    //     backRight.spin(fwd);
    //     turnTo(currentAbsoluteOrientation + (((visionSensor.largestObject.centerX - 157.5)/157.5) * M_PI/6), 2500);
    //   } else if (visionSensor.largestObject.centerX < 60) {
    //     // turn right
    //     // turnTo(currentAbsoluteOrientation + M_PI/2, 1000);
    //     frontLeft.spin(fwd);
    //     frontRight.spin(reverse);
    //     backLeft.spin(fwd);
    //     backRight.spin(reverse);
    //   } else if (visionSensor.largestObject.centerX > 60 && visionSensor.largestObject.centerX < 255) {
    //     if (visionSensor.largestObject.width < 160) {
    //       //drive forward
    //       // driveTo(globalX + ammount, globalY + ammount, currentAbsoluteOrientation, 1000, 1);
    //       // turnTo(currentAbsoluteOrientation, 1000);
    //       frontLeft.spin(reverse);
    //       frontRight.spin(reverse);
    //       backLeft.spin(reverse);
    //       backRight.spin(reverse);
    //     } else {
    //       frontLeft.stop(coast);
    //       frontRight.stop(coast);
    //       backLeft.stop(coast);
    //       backRight.stop(coast);
    //     }
    //   }
    // } else {
    //   // turnTo(currentAbsoluteOrientation + M_PI/2, 1000); //make the robot turn, could just not use PID but try it with this
    //   frontLeft.spin(fwd);
    //   frontRight.spin(reverse);
    //   backLeft.spin(fwd);
    //   backRight.spin(reverse);
    // }

    if (visionSensor.objectCount > 0) {
      driveTo(globalX - (cos(currentAbsoluteOrientation) * visionSensor.largestObject.centerY * 0.01125), globalY - (sin(currentAbsoluteOrientation) * visionSensor.largestObject.centerY * 0.01125), currentAbsoluteOrientation, 2500, 1);
      if (visionSensor.largestObject.centerX > 255 && visionSensor.largestObject.centerX < 60) {
        turnTo(currentAbsoluteOrientation + (((visionSensor.largestObject.centerX - 157.5)/157.5) * M_PI/6), 2500);
        //try just mkaing a visionSensor PID which uses the visionSesnor pixels directly instead of converting it to meters, do the same for Pure Pursuit. Also like maybe have a somewhat constant drive velocity (like pure pursuit?)
      }
    } else {
      // turnTo(currentAbsoluteOrientation + M_PI, 2500);
      frontLeft.spin(fwd);
      frontRight.spin(reverse);
      backLeft.spin(fwd);
      backRight.spin(reverse);
    }
    task::sleep(40);
  }
}
