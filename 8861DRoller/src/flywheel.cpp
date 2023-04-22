#include "flywheel.h"
bool flyWheelOn = false;
float flyWheelSpeed = 460; // 80 for pct, i am using 10 for voltage 

// float kI = 0.00025; //again, this is arbitrary
float flyWheel1PowerTBH = 0;
float flyWheel1Error = 0;
float prevFlyWheel1Error = 0;
float TBH1 = 0;
float prevFlyWheel1RPM = 0;
bool flyWheel1FirstCross = false;

float flyWheel2PowerTBH = 0;
float flyWheel2Error = 0;
float prevFlyWheel2Error = 0;
float TBH2 = 0;
float prevFlyWheel2RPM = 0;
bool flyWheel2FirstCross = false;


float kP = 0.0005; //again, this is arbitrary
float kI = 0.00025; //again, this is arbitrary
float flyWheelPowerPIC = 0;
float flyWheelError = 0;
// float prevFlyWheelError1 = (flyWheel1.velocity(rpm) + flyWheel2.velocity(rpm))/2; 
// float prevFlyWheelError2 = prevFlyWheelError1; 
// float prevFlyWheelError3 = prevFlyWheelError2; 
// float prevFlyWheelError4 = prevFlyWheelError3; 
float flyWheelInegral = 0;

void increaseFlyWheelSpeed() {
  if (flyWheelSpeed < 600) {  // < 100 for pct 12 for voltage 
    flyWheelSpeed += 25;
    flyWheelPowerPIC = flyWheelSpeed/600;
  }
}

void decreaseFlyWheelSpeed() {
  if (flyWheelSpeed > 350) { // 6
    flyWheelSpeed -= 25; // 0.1
    flyWheelPowerPIC = flyWheelSpeed/600;
  }
}

int sgnTBH (float num) {
  if (num >= 0)
    return 1;
  else
    return -1;
}

int flyWheelPICTask() {
  flyWheel.setStopping(coast);
  timer flyWheelTimer = timer();
  while (true) {
    float approxPower = flyWheelSpeed/600;
    if (flyWheelOn == true) {
      flyWheelError = flyWheelSpeed - (flyWheel.velocity(rpm)); //plus because the rpm is negative;
      if (fabs(flyWheelError) > 10) {
        flyWheelPowerPIC += 0.000875 * sgnTBH(flyWheelError);
      } else 
      
      if(flyWheelPowerPIC > 1) {
        flyWheelPowerPIC = 1;
      } else if(flyWheelPowerPIC < 0) {
        //Keep the motor power positive, to prevent damaging gears or motors
        flyWheelPowerPIC = 0;
      }

      // printf("\n %f", flyWheelPowerPIC);
      flyWheel.spin(directionType::fwd, (flyWheelPowerPIC) * 12, voltageUnits::volt);

      task::sleep(50);
    }
    else
    {
      flyWheelTimer.reset();
      flyWheel.stop(coast);
    }
  }
  return 1;
}

void toggleFlyWheel() {
  if (flyWheelOn == false) {
    flyWheelPowerPIC = flyWheelSpeed/600;
    flyWheelOn = true;
  } else if (flyWheelOn == true) {
    flyWheel.stop(coast);
    flyWheelOn = false;
  }
}