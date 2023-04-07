#include "intake.h"

bool intakeOn = false;
int intakeSpeed = 100;

// void rollerRed() {
//   intake.spinFor(reverse, 0.3, rev, 100, velocityUnits::pct, true);
//   // wait(100, msec);
//   intakeOn = true;
//   timer rollerTimeout = timer();
//   opticalSensor.setLightPower(100);
//   opticalSensor.setLight(vex::ledState::on);
//   while (opticalSensor.color() != color(blue) && rollerTimeout.time() < 2500 /*&& opticalSensor.isNearObject()*/) {
//     intake.spin(reverse, 20 /*- (rollerTimeout.time(msec)/125)*/, velocityUnits::pct);
//     frontLeft.spin(reverse, 0.5, voltageUnits::volt);
//     frontRight.spin(reverse, 0.5, voltageUnits::volt);
//     backLeft.spin(reverse, 0.5, voltageUnits::volt);
//     backRight.spin(reverse, 0.5, voltageUnits::volt);
//     wait(5, msec);
//   }
//   intake.stop();
//   intakeOn = false;

//   opticalSensor.setLight(vex::ledState::off);
// }

void toggleIntake() {
  if (intakeOn == false) {
    intake.spin(fwd, intakeSpeed, velocityUnits::pct);
    intakeOn = true;
  } else if (intakeOn == true) {
    intake.stop();
    intakeOn = false;
  }
}