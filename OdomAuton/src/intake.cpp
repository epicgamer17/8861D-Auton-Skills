#include "intake.h"

bool intakeOn = false;

// void turnroller() {
//   // PI/DFunc(-0.05, 0, 0);
//   intake.spinFor(reverse, 1, rev, 100, velocityUnits::pct, true);
// }

void rollerSingleRotation() {
  intake.spinFor(reverse, 1, rev, 100, velocityUnits::pct, true);
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