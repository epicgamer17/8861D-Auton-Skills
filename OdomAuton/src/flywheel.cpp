#include "flywheel.h"

bool flyWheelOn = false;
int flyWheelSpeed = 10; // 80 for pct, i am using 10 for voltage 

#pragma region PID Varialbes
// float desiredVoltage = 0;

double flyWheelMaxError = 0.1;

double flyWheelIntegralBound = 1.5;

double flyWheelkP = 10;
double flyWheelkI = 0.0;
double flyWheelkD = 0.0;

double flyWheelPowerPID = 0;

float flyWheelError; //Desired Value - Sensor Value: Position
float flyWheelPrevError = 0; //Position 20ms ago
float flyWheelDerivative; // error - prevError : Speed
float flyWheelIntegral = 0; // totalError += error : Integral 

#pragma endregion PID Variables

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


void flyWheelPID() {
  float avgVoltage = (flyWheel1.voltage() + flyWheel2.voltage())/2; //could add previous voltages too 
  //Error is equal to the total distance away from the target (uses distance formula with current position and target location)
  flyWheelError = flyWheelSpeed - avgVoltage;
  
  //only use integral if close enough to target
  if(fabs(flyWheelError) < flyWheelIntegralBound) {
    flyWheelIntegral += flyWheelError;
  }
  else {
    flyWheelIntegral = 0;
  }

  //reset integral if we pass the target
  if(flyWheelError * flyWheelPrevError < 0) {
    flyWheelIntegral = 0;
  } 

  flyWheelDerivative = flyWheelError - flyWheelPrevError;

  flyWheelPrevError = flyWheelError;

  flyWheelPowerPID = (flyWheelError * flyWheelkP + flyWheelIntegral * flyWheelkI + flyWheelDerivative * flyWheelkD);

  //Limit power output to 12V
  if(flyWheelPowerPID > 12) {
    flyWheelPowerPID = 12;
  }

  if(fabs(flyWheelError) < flyWheelMaxError) {
    flyWheelPowerPID = 0;
  }
}

int flyWheelPIDTask() {

  //loop to constantly execute chassis commands
  while(1) {
    
    if(flyWheelOn) {
      //get PID values for driving and turning
      flyWheelPID();

      //set power for each motor
      flyWheel1.spin(reverse, ((flyWheelPowerPID)), voltageUnits::volt);
      flyWheel2.spin(reverse, ((flyWheelPowerPID)), voltageUnits::volt);

    }
    //What to do when not using the chassis controls
    else {
      flyWheel1.stop(brakeType::coast);
      flyWheel2.stop(brakeType::coast);
    }

  }

  return 1;
}

void toggleFlyWheel() {
  if (flyWheelOn == false) {
    flyWheelSpeed = 12;
    // flyWheel1.spin(directionType::fwd, -flyWheelSpeed, voltageUnits::volt);
    // flyWheel2.spin(directionType::fwd, -flyWheelSpeed, voltageUnits::volt);
    printf("%d", flyWheelOn);
    flyWheelOn = true;
  } else if (flyWheelOn == true) {
    // flyWheel1.stop();
    // flyWheel2.stop();
    flyWheelOn = false;
    printf("%d", flyWheelOn);
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
