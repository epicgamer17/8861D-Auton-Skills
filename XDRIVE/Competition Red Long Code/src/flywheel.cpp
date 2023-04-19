#include "flywheel.h"
bool flyWheelOn = false;
float flyWheelSpeed = 480; // 80 for pct, i am using 10 for voltage 

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

//Index Function
void index() {
  indexer.startSpinFor(1, rev, 100, velocityUnits::pct);
}

void increaseFlyWheelSpeed() {
  // if (flyWheelSpeed < 12) {  // < 100 for pct 12 for voltage 
  //   flyWheelSpeed += 0.1;
  //   if (flyWheelOn == true) {
  //     // flyWheel1.spin(reverse, flyWheelSpeed, voltageUnits::volt);
  //     // flyWheel2.spin(reverse, flyWheelSpeed, voltageUnits::volt);
  //     // flyWheel1FirstCross = true;
  //     // flyWheel2FirstCross = true;
  //   }
  // }
  if (flyWheelSpeed < 590) {  // < 100 for pct 12 for voltage 
    flyWheelSpeed += 5;
    flyWheelPowerPIC = flyWheelSpeed/600;
  }
}

void decreaseFlyWheelSpeed() {
  // if (flyWheelSpeed > 6) { // 6
  //   flyWheelSpeed -= 0.1; // 0.1
  //   if (flyWheelOn == true) {
  //     // flyWheel1.spin(reverse, flyWheelSpeed, voltageUnits::volt);
  //     // flyWheel2.spin(reverse, flyWheelSpeed, voltageUnits::volt);
  //     // flyWheel1FirstCross = true;
  //     // flyWheel2FirstCross = true;
  //   }
  // }

  if (flyWheelSpeed > 350) { // 6
    flyWheelSpeed -= 5; // 0.1
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
  // flyWheel1.setBrake(coast);
  // flyWheel2.setBrake(coast);

  flyWheel.setStopping(coast);
  timer flyWheelTimer = timer();
  while (true) {
    float approxPower = flyWheelSpeed/600;
    if (flyWheelOn == true) {
      // flyWheelError = flyWheelSpeed - (flyWheel1.velocity(rpm) + flyWheel2.velocity(rpm))/2; //plus because the rpm is negative;
      flyWheelError = flyWheelSpeed - (flyWheel.velocity(rpm)); //plus because the rpm is negative;
      // average out flyWheelErrors for a smoother flywheel error 
      // flyWheelError = flyWheelSpeed - (((flyWheel1.velocity(rpm) + flyWheel2.velocity(rpm))/2) + prevFlyWheelError1 + prevFlyWheelError2 + prevFlyWheelError3 + prevFlyWheelError4)/5; //plus because the rpm is negative;
      // prevFlyWheelError4 = prevFlyWheelError3; 
      // prevFlyWheelError3 = prevFlyWheelError2; 
      // prevFlyWheelError2 = prevFlyWheelError1; 
      // prevFlyWheelError1 = (flyWheel1.velocity(rpm) + flyWheel2.velocity(rpm))/2; 
      
      // if (flyWheelError < 20) {
      //   flyWheelInegral += flyWheelError;
      // } else {
      //   flyWheelInegral = 0;
      // }

      if (fabs(flyWheelError) > flyWheelSpeed/50 && fabs(flyWheelError) < flyWheelSpeed/25 + flyWheelTimer.time(sec) * 2) {

        // flyWheelPowerPIC = approxPower + (flyWheelError * kP) + (flyWheelInegral * kI);
        
        // flyWheelPowerPIC += (flyWheelError * kP);
        
        flyWheelPowerPIC += 0.001875 * sgnTBH(flyWheelError);
      } else 
      
      if(flyWheelPowerPIC > 1) {
        flyWheelPowerPIC = 1;
      } else if(flyWheelPowerPIC < 0) {
        //Keep the motor power positive, to prevent damaging gears or motors
        flyWheelPowerPIC = 0;
      }
      
      // printf("\n Flywheel Error %f", flyWheelError);
      // printf("\n Flywheel Power PIC %f", flyWheelPowerPIC);
      // flyWheel1.spin(directionType::fwd, (1) * 12, voltageUnits::volt);
      // flyWheel2.spin(directionType::fwd, (1) * 12, voltageUnits::volt);

      flyWheel.spin(directionType::fwd, (flyWheelPowerPIC) * 12, voltageUnits::volt);

      task::sleep(25);
    }
    else
    {
      // flyWheel1.stop(coast);
      // flyWheel2.stop(coast);
      flyWheelTimer.reset();
      flyWheel.stop(coast);
    }
  }
  return 1;
}

int flyWheelTBHTask() {
//   // timer flyWheelTimer = timer();
//   flyWheel1.setBrake(coast);
//   flyWheel2.setBrake(coast);
//   while (true) {
//     float approxPower = flyWheelSpeed/565;
//     if (flyWheelOn == true) {
//       flyWheel1Error = flyWheelSpeed - flyWheel1.velocity(rpm); //plus because the rpm is negative;
//       flyWheel1PowerTBH += (flyWheel1Error * kI);
      
//       if(flyWheel1PowerTBH > 1) {
//         flyWheel1PowerTBH = 1;
//       }
//       else if(flyWheel1PowerTBH < 0) {
//       //Keep the motor power positive, to prevent damaging gears or motors
//         flyWheel1PowerTBH = 0;
//       }
      

//       if(sgnTBH(flyWheel1Error) != sgnTBH(prevFlyWheel1Error)) {
//         if (flyWheel1FirstCross) {
//           flyWheel1PowerTBH = approxPower; 
//           flyWheel1FirstCross = false;
//         } else {
//           flyWheel1PowerTBH = 0.5 * (flyWheel1PowerTBH + TBH1);
//           // TBH1 = (flyWheel1PowerTBH + TBH1) / 2;
//           // flyWheel1PowerTBH = TBH1;
//           // prevFlyWheel1Error = flyWheel1Error;
//         }
//         TBH1 = flyWheel1PowerTBH;      
//       }
      

//       prevFlyWheel1Error = flyWheel1Error;   

//       flyWheel2Error = flyWheelSpeed - flyWheel2.velocity(rpm); //plus because the rpm is negative;
//       flyWheel2PowerTBH += (flyWheel2Error * kI);
      
//       if(flyWheel2PowerTBH > 1) {
//         flyWheel2PowerTBH = 1;
//       }
//       else if(flyWheel2PowerTBH < 0) {
//       //Keep the motor power positive, to prevent damaging gears or motors
//         flyWheel1PowerTBH = 0;
//       }
      
//       if(sgnTBH(flyWheel2Error) != sgnTBH(prevFlyWheel2Error)) {
//         if (flyWheel2FirstCross) {
//           flyWheel2PowerTBH = approxPower; 
//           flyWheel2FirstCross = false;
//         } else {
//           flyWheel2PowerTBH = 0.5 * (flyWheel2PowerTBH + TBH2);
//           // TBH1 = (flyWheel1PowerTBH + TBH1) / 2;
//           // flyWheel1PowerTBH = TBH1;
//           // prevFlyWheel1Error = flyWheel1Error;
//         }
//         TBH2 = flyWheel2PowerTBH;      
//       }
      
//       prevFlyWheel2Error = flyWheel2Error;  


//       // flyWheel1.spin(directionType::fwd, (flyWheel1PowerTBH) * 12, voltageUnits::volt);
//       // flyWheel2.spin(directionType::fwd, (flyWheel2PowerTBH) * 12, voltageUnits::volt);

//       flyWheel.spin(directionType::fwd, (flyWheel1PowerTBH) * 12, voltageUnits::volt);

//       task::sleep(50);
//     }
//     else
//     {
//       // flyWheel1.stop(coast);
//       // flyWheel2.stop(coast);

//       flyWheel.stop(coast);

//     }
//   }

  return 1;
}

void toggleFlyWheel() {
  if (flyWheelOn == false) {
    // flyWheel1.spin(fwd, flyWheelSpeed, voltageUnits::volt);
    // flyWheel2.spin(fwd, flyWheelSpeed, voltageUnits::volt);
    // flyWheel1FirstCross = true;
    // flyWheel2FirstCross = true;
    
    flyWheelPowerPIC = flyWheelSpeed/600;
    flyWheelOn = true;
  } else if (flyWheelOn == true) {
    // flyWheel1.stop(coast);
    // flyWheel2.stop(coast);

    flyWheel.stop(coast);

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