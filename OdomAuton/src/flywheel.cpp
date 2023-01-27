#include "flywheel.h"

bool flyWheelOn = false;
float flyWheelSpeed = 10; // 80 for pct, i am using 10 for voltage 

//Index Function
void index() {
  indexer.startSpinFor(1, rev, 100, velocityUnits::pct);
}

void increaseFlyWheelSpeed() {
  if (flyWheelSpeed < 12) {  // < 100 for pct 12 for voltage 
    flyWheelSpeed += 0.5;
  }
}

void decreaseFlyWheelSpeed() {
  if (flyWheelSpeed > 0) {
    flyWheelSpeed -= 0.5;
  }
}


float kI = .025; //again, this is arbitrary
float flyWheelPowerTBH = 0;
// float deltaFlyWheelRPM = 0;
float flyWheelError = 0;
float prevFlyWheelError = 0;
float flyWheelIntegral = 0;
float TBH = 0;
float flyWheelTime = 0;
float prevFlyWheelRPM = 0;
float prevFlyWheelTime = 0;


int sgn (float num) {
  if (num >= 0)
    return 1;
  else
    return -1;
}

int TBHTask() {
  timer flyWheelTimer = timer();
  while (true) {
    if (flyWheelOn == true) {
      flyWheelTime = flyWheelTimer.time(msec);
      // since rpm is already sort of deltaPosition we maybe can just use rpm instead of chage in degrees// deltaFlyWheelRPM = (flyWheel1.velocity(rpm) - prevFlyWheelRPM) / (flyWheelTime - prevFlyWheelTime);
      flyWheelError = flyWheelSpeed - flyWheel1.velocity(rpm);
      flyWheelIntegral += flyWheelError;
      flyWheelPowerTBH = flyWheelIntegral * kI;
      
      prevFlyWheelTime = flyWheelTime;

      if(flyWheelPowerTBH > 12) {
        flyWheelPowerTBH = 12;
      }
      else if(flyWheelPowerTBH < 0) {
      //Keep the motor power positive, to prevent damaging gears or motors
        flyWheelPowerTBH = 0;
      }
      
      if(sgn(flyWheelError) != sgn(prevFlyWheelError)) {
        //If the sign of the error changes, then the error crossed zero
        TBH = (flyWheelPowerTBH + TBH) / 2;
        flyWheelPowerTBH = TBH;
        prevFlyWheelError = flyWheelError;
        //the last error doesn't matter unless the sign is different, so the last error is only stored when necessary
      }

      flyWheel1.spin(directionType::fwd, -flyWheelPowerTBH, voltageUnits::volt);
      flyWheel2.spin(directionType::fwd, -flyWheelPowerTBH, voltageUnits::volt);

      task::sleep(20);
    }
    else
    {
      flyWheel1.stop(coast);
      flyWheel2.stop(coast);
    }
  }
}

void toggleFlyWheel() {
  if (flyWheelOn == false) {
    // flyWheel1.spin(directionType::fwd, -flyWheelSpeed, voltageUnits::volt);
    // flyWheel2.spin(directionType::fwd, -flyWheelSpeed, voltageUnits::volt);
    flyWheelOn = true;
  } else if (flyWheelOn == true) {
    // flyWheel1.stop(coast);
    // flyWheel2.stop(coast);
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