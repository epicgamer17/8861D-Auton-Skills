#include "PID.h"

#pragma region PID Varialbes
bool enablePID = true;
bool resetEncoders = false;

float desiredX = 0;
float desiredY = 0;
float desiredHeading = 0;
float timeoutLength = 2500;

//front left + back right drive power
float drivePowerFLBR = 0;
//front right + back left drive power
float drivePowerFRBL = 0;

// float kP = 100;  //0.32 best for no tracking wheels and only p//0.5 might be better // or 0.25
// float kI = 0.00; //0.00025 might be better // or 0.00125
// float kD = 0.0; //0.005 might be better //or 0.0025
// float turnkP = 0.5; // 0.1 is really nice, at least for only P values and no intertial 
//                   //0.5 or 0.6 seems to work for only P values with intertial but you need to add +1 to your angle
//                   //2 also work for intertial for turning
// float turnkI = 0.0; 
// float turnkD = 0.0; //might be better to have this low so that it does zig zag when driving straight (worth testing though)

double turnError = 0;
double turnPrevError = 0;

double turnMaxError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.09;

double turnDerivative = 0;

double turnkP = 13.00;
double turnkI = 1.00;
double turnkD = 10.00;

double turnPowerPID = 0;

// double driveError = 0;
// double drivePrevError = 0;

double driveMaxError = 0.1;

// double driveIntegral = 0;
double driveIntegralBound = 1.5;

// double driveDerivative = 0;

double drivekP = 1.5;
double drivekI = 0.02;
double drivekD = 10.0;

double drivePowerPID = 0;

float driveError; //Desired Value - Sensor Value: Position
float drivePrevError = 0; //Position 20ms ago
float driveDerivative; // error - prevError : Speed
float driveIntegral = 0; // totalError += error : Integral 



// double yDrivePowerPID = 0;
// double xDrivePowerPID = 0;

// float yDriveError; //Desired Value - Sensor Value: Position
// float yDrivePrevError = 0; //Position 20ms ago
// float yDriveDerivative; // error - prevError : Speed
// float yDriveIntegral = 0; // totalError += error : Integral 

// float xDriveError; //Desired Value - Sensor Value: Position
// float xDrivePrevError = 0; //Position 20ms ago
// float xDriveDerivative; // error - prevError : Speed
// float xDriveIntegral = 0; // totalError += error : Integral 

// float turnError; //Desired Value - Sensor Value: Position
// float turnPrevError = 0; //Position 20ms ago
// float turnDerivative; // error - prevError : Speed
// float turnTotalError = 0; // totalError += error : Integral 

// int maxTurnIntegral = 20; // These cap the integrals
// int maxIntegral = 3000;
// int integralBound = 3; //If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees
int maxSpeed = 1; //60 max speed good for tank drive 
int maxTurningSpeed = 1;
#pragma endregion PID Variables


void driveTo(float dX, float dY, float dH, float timeoutTime = 2500, float mSpeed = 1.0) {  // COULD TRY PID WITH VOLTAGE INSTEAD
  desiredX = dX;
  desiredY = dY;
  desiredHeading = dH;
  enablePID = true;
  timeoutLength = timeoutTime;
  Brain.resetTimer();
  maxSpeed = mSpeed;
}

void turnTo(float dH, float timeoutTime = 2500) {
  desiredHeading = dH;
  desiredX = globalX; 
  desiredY = globalY;

  timeoutLength = timeoutTime;

  Brain.resetTimer();
}

void turnToPoint(float dX, float dY, float timeoutTime = 2500) {
  desiredHeading = atan2(dY - globalY, dX - globalX);

  if (desiredHeading < 0) {
    desiredHeading = 2 * M_PI - fabs(desiredHeading);
  }

  desiredX = globalX; 
  desiredY = globalY;

  timeoutLength = timeoutTime;

  Brain.resetTimer();
}

void setDrivePower(float theta) {
  drivePowerFLBR = sin(theta + M_PI_4) / sin(M_PI_4);

  //Limits the value to 1
  if(fabs(drivePowerFLBR) > 1) {
    drivePowerFLBR = fabs(drivePowerFLBR) / drivePowerFLBR;
  }

  drivePowerFRBL = sin(theta - M_PI_4) / sin(M_PI_4);

  //Limits the value to 1
  if(fabs(drivePowerFRBL) > 1) {
    drivePowerFRBL = fabs(drivePowerFRBL) / drivePowerFRBL;
  }
}

void drivePID() {
  
  //Error is equal to the total distance away from the target (uses distance formula with current position and target location)
  driveError = sqrt(pow((globalX - desiredX), 2) + pow((globalY - desiredY), 2));
  
  //only use integral if close enough to target
  if(fabs(driveError) < driveIntegralBound) {
    driveIntegral += driveError;
  }
  else {
    driveIntegral = 0;
  }

  //reset integral if we pass the target
  if(driveError * drivePrevError < 0) {
    driveIntegral = 0;
  } 

  driveDerivative = driveError - drivePrevError;

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  //Limit power output to 12V
  if(drivePowerPID > 12) {
    drivePowerPID = 12;
  }

  if(fabs(driveError) < driveMaxError) {
    drivePowerPID = 0;
  }

}


void turnPID() {
  //Error is equal to the difference between the current facing direction and the target direction
  turnError = currentAbsoluteOrientation - desiredHeading;

  if(fabs(turnError) > M_PI) {
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * M_PI - turnError);
  }

  //only use integral if close enough to target
  if(fabs(turnError) < turnIntegralBound) {
    turnIntegral += turnError;
  }
  else {
    turnIntegral = 0;
  }

  //reset integral if we pass the target
  if(turnError * turnPrevError < 0) {
    turnIntegral = 0;
  } 

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPowerPID = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  //Limit power output to 12V
  if(turnPowerPID > 12) {
    turnPowerPID = 12;
  }

  if(fabs(turnError) < turnMaxError) {
    turnPowerPID = 0;
  }
}


double frontLeftPower = 0;
double frontRightPower = 0;
double backLeftPower = 0;
double backRightPower = 0;

//distances between robot's current position and the target position
double xDistToTarget = 0;
double yDistToTarget = 0;

//angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

/* CHASSIS CONTROL TASK */
int PIDTask() {

  //loop to constantly execute chassis commands
  while(1) {
    
    if(enablePID) {
      //Distances to target on each axis
      xDistToTarget = desiredX - globalX;
      yDistToTarget = desiredY - globalY;

      //Angle of hypotenuse
      hypotenuseAngle = atan2(yDistToTarget, xDistToTarget);

      if(hypotenuseAngle < 0) {
        hypotenuseAngle += 2 * M_PI;
      }

      //The angle the robot needs to travel relative to its forward direction in order to go toward the target
      robotRelativeAngle = hypotenuseAngle - currentAbsoluteOrientation + M_PI_2;

      if(robotRelativeAngle > 2 * M_PI) {
        robotRelativeAngle -= 2 * M_PI;
      }
      else if(robotRelativeAngle < 0) {
        robotRelativeAngle += 2 * M_PI;
      }

      //Get the power percentage values for each set of motors
      setDrivePower(robotRelativeAngle);

      //get PID values for driving and turning
      drivePID();
      turnPID();

      //set power for each motor
      frontLeftPower = ((drivePowerFLBR * drivePowerPID) + turnPowerPID) * maxSpeed;
      frontRightPower = ((drivePowerFRBL * drivePowerPID) - turnPowerPID) * maxSpeed;
      backLeftPower = ((drivePowerFRBL * drivePowerPID) + turnPowerPID) * maxSpeed;
      backRightPower = ((drivePowerFLBR * drivePowerPID) - turnPowerPID) * maxSpeed;
      // frontLeft.spin(fwd, xDrivePowerPID + turnPowerPID, velocityUnits::pct); // could use voltage
      // backLeft.spin(fwd, yDrivePowerPID + turnPowerPID , velocityUnits::pct); // could use voltage
      // frontRight.spin(fwd, xDrivePowerPID - turnPowerPID, velocityUnits::pct); // could use voltage
      // backRight.spin(fwd, yDrivePowerPID - turnPowerPID, velocityUnits::pct); // could use voltage


      frontLeft.spin(directionType::fwd, frontLeftPower, voltageUnits::volt);
      frontRight.spin(directionType::fwd, frontRightPower, voltageUnits::volt);
      backLeft.spin(directionType::fwd, backLeftPower, voltageUnits::volt);
      backRight.spin(directionType::fwd, backRightPower, voltageUnits::volt);
   
      if(fabs(driveError) < 0.1 && fabs(turnError) < 0.003) {
        enablePID = false;
      }

      if(Brain.timer(timeUnits::msec) > timeoutLength) {
        enablePID = false;
      }

      // Brain.Screen.setCursor(1,2);
      // Brain.Screen.print("Hyp angle: %f", hypotenuseAngle);
      // Brain.Screen.setCursor(2,2);
      // Brain.Screen.print(FrontLeftPower);
      //Brain.Screen.print("botRelativeAngle: %f", robotRelativeAngle);

      // Brain.Screen.setCursor(4,2);
      // Brain.Screen.print("absolute orientation: %f", currentAbsoluteOrientation);
      // // Brain.Screen.setCursor(3,2);
      // Brain.Screen.print(drivePowerFLBR);

    }
    //What to do when not using the chassis controls
    else {
      // FrontLeftDrive.stop(brakeType::brake);
      // FrontRightDrive.stop(brakeType::brake);
      // BackLeftDrive.stop(brakeType::brake);
      // BackRightDrive.stop(brakeType::brake);
      frontLeft.stop(brakeType::coast);
      frontRight.stop(brakeType::coast);
      backLeft.stop(brakeType::coast);
      backRight.stop(brakeType::coast);
    }
    
    task::sleep(20);

  }

  return 1;
}

// void PID() {
//   while (enablePID) {
//     xError = desiredX - globalX;
//     yError = desiredY - globalY;
//     float inertialRadians = (inertialSensor.heading() - 45) * (M_PI/180);
//     float temp = yError * cos(inertialRadians) + xError * sin(inertialRadians);
//     xError = yError * sin(inertialRadians) + xError * cos(inertialRadians);
//     yError = temp;

//     // Derivative 
//     yDerivative =  yError - yPrevError; //may need to flip // is this equal to ticks travelled? 

//     // Velocity -> Position -> Absement // Integral
//     //Integral
//     if(fabs(yError) < integralBound){
//     yTotalError+=yError; 
//     }  else {
//       yTotalError = 0; 
//     }
//     //totalError += error;

//     //This would cap the integral
//     yTotalError = fabs(yTotalError) > maxIntegral ? signnum_c(yTotalError) * maxIntegral : yTotalError;

//     // Calculate motor power
//     double yMotorPower = (yError * kP) + (yDerivative * kD) + (yTotalError * kI); 
//     //   pidDrive = (pid_Kp * pidError);

//     // Derivative 
//     xDerivative =  xError - xPrevError; //may need to flip // is this equal to ticks travelled? 

//     // Velocity -> Position -> Absement // Integral
//     //Integral
//     if(fabs(xError) < integralBound){
//     xTotalError+=xError; 
//     }  else {
//       xTotalError = 0; 
//     }
//     //totalError += error;

//     //This would cap the integral
//     xTotalError = fabs(xTotalError) > maxIntegral ? signnum_c(xTotalError) * maxIntegral : xTotalError;

//     // Calculate motor power
//     double xMotorPower = (xError * kP) + (xDerivative * kD) + (xTotalError * kI); 


//     frontLeft.spin(fwd, xMotorPower + turnMotorPower, velocityUnits::pct); // could use voltage
//     backLeft.spin(fwd, yMotorPower + turnMotorPower , velocityUnits::pct); // could use voltage
//     frontRight.spin(fwd, yMotorPower - turnMotorPower, velocityUnits::pct); // could use voltage
//     backRight.spin(fwd, xMotorPower - turnMotorPower, velocityUnits::pct); // could use voltage


//     yPrevError = yError;
//     xPrevError = xError;
//     turnPrevError = turnError;

//     if ((fabs(yError) < 0.03 && fabs(xError) < 0.03) && (fabs(turnError) < 1)) // && ((desiredValue != 0) || (desiredTurnValue != 0))
//     {
//       // resetEncoders = true;
//       return; 
//     }
    
//     Brain.Screen.clearScreen();
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.print("Y Error: %f ", yError);
//     Brain.Screen.newLine();
//     Brain.Screen.print("X Error: %f ", xError);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Turn Error: %f ", turnError);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Desired X: %f ", desiredX);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Current X: %f ", globalX);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Desired Y: %f ", desiredY);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Current Y: %f ", globalY);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Desired Turn: %f ", desiredHeading);
//     Brain.Screen.newLine();
//     Brain.Screen.print("Inertial: %f", inertialSensor.rotation());
//     Brain.Screen.newLine();
//     Brain.Screen.print("Failsafe Timer: %d", timeout.time());
//     Brain.Screen.newLine();
//     // Don't hog cpu
//     vex::task::sleep(20);
//   }

//   return;
// }



// xDriveError = desiredX - globalX;
//   yDriveError = desiredY - globalY;
//   // driveError = sqrt(pow((globalX - desiredX), 2) + pow((globalX - desiredX), 2));

//   float inertialRadians = (inertialSensor.heading() - 45) * (M_PI/180);
//   float temp = yDriveError * cos(inertialRadians) + xDriveError * sin(inertialRadians);
//   xDriveError = yDriveError * sin(inertialRadians) + xDriveError * cos(inertialRadians);
//   yDriveError = temp;

//   ////////////////////
//   // Y
//   ////////////////////
//   // Velocity -> Position -> Absement // Integral
//   //Integral
//   if(fabs(yDriveError) < driveIntegralBound){
//   yDriveIntegral += yDriveError; 
//   } else {
//     yDriveIntegral = 0; 
//   }
//   //totalError += error;

//   //reset integral if we pass the target
//   if(yDriveError * yDrivePrevError < 0) {
//     yDriveIntegral = 0;
//   } 

//   //only use integral if close enough to target

//   yDriveDerivative = yDriveError - yDrivePrevError;

//   yDrivePrevError = yDriveError;

//   yDrivePowerPID = (yDriveError * drivekP + yDriveIntegral * drivekI + yDriveDerivative * drivekD);


//   ////////////////////
//   // X
//   ////////////////////
//   // Velocity -> Position -> Absement // Integral
//   //Integral
//   if(fabs(xDriveError) < driveIntegralBound){
//   xDriveIntegral += xDriveError; 
//   } else {
//     xDriveIntegral = 0; 
//   }
//   //totalError += error;

//   //reset integral if we pass the target
//   if(xDriveError * xDrivePrevError < 0) {
//     xDriveIntegral = 0;
//   } 

//   //only use integral if close enough to target

//   xDriveDerivative = xDriveError - xDrivePrevError;

//   xDrivePrevError = xDriveError;

//   xDrivePowerPID = (xDriveError * drivekP + xDriveIntegral * drivekI + xDriveDerivative * drivekD);


//   if(xDriveError * xDrivePrevError < 0) {
//     xDriveIntegral = 0;
//   } 

//   //Limit power output to 12V
//   if(xDrivePowerPID > 12) {
//     xDrivePowerPID = 12;
//   }

//   if(fabs(xDriveError) < driveMaxError) {
//     xDrivePowerPID = 0;
//   }

//   //Limit power output to 12V
//   if(yDrivePowerPID > 12) {
//     yDrivePowerPID = 12;
//   }

//   if(fabs(yDriveError) < driveMaxError) {
//     yDrivePowerPID = 0;
//   }
