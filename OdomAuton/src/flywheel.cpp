#include "flywheel.h"

bool flyWheelOn = false;
int flyWheelSpeed = 10; // 80 for pct, i am using 10 for voltage 

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


// void aligngoal(std::string colour) {
//   char *string = &colour[0];
//   vision::signature *col;
  
//   if (strcmp(string, "blue")) {
//     col = &BLUEGL;
//   }
//   else if (strcmp(string, "red")) {
//     col = &REDGL;
//   }
//   else {
//     col = &DISC;
//   }
  
//   double point[] = {0,0};
//   vision1.takeSnapshot(*col);
//   if(vision1.largestObject.exists) {
//     point[0] = vision1.largestObject.originX;
//     point[1] = vision1.largestObject.originY;
//   }
// }
