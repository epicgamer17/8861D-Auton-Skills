#include "PID.h"

bool fieldOriented = false;
bool correctingPosition = false;

#pragma region PID Varialbes
bool enablePID = true;
bool resetEncoders = false;
bool userControl = false;
bool turningToPoint = false;
bool invertedTurning = false;
bool useDistance = false; 

float desiredX = 0;
float desiredY = 0;
float desiredTurnX = 0;
float desiredTurnY = 0;
float desiredHeading = 0;
float timeoutLength = 2500;

//front left + back right drive power
float drivePowerFLBR = 0;
//front right + back left drive power
float drivePowerFRBL = 0;

double turnError = 0;
double turnPrevError = 0;

double turnMinError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.09; //i think the units are radians so no need to convert to radians 

double turnDerivative = 0;

// double turnkP = 10;
// double turnkI = 0.00;
// double turnkD = 0.00;

double turnkP = 16.75; //13
double turnkI = 3.00; //1
double turnkD = 10.00; // 10
/// T Ultimate = 0.3875s 
/// K Ultimate = 73
double turnPowerPID = 0;

double driveMinError = 0.0025;

double driveIntegralBound = 0.038; //converted inches to m (23880E had 1.5 inches)

// double drivekP = 20;
// double drivekI = 0.0;
// double drivekD = 0.0;

double drivekP = 95; //converted to meters. not
double drivekI = 1.5; //converted to meters
double drivekD = 394; //converted to meters

double drivePowerPID = 0;

float driveError; //Desired Value - Sensor Value: Position
float drivePrevError = 0; //Position 20ms ago
float driveDerivative; // error - prevError : Speed
float driveIntegral = 0; // totalError += error : Integral 

float frontLeftPower = 0;
float frontRightPower = 0;
float backLeftPower = 0;
float backRightPower = 0;

//distances between robot's current position and the target position
double xDistToTarget = 0;
double yDistToTarget = 0;

//angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

float maxSpeed = 1; //60 max speed good for tank drive 
float maxTurningSpeed = 1;
#pragma endregion PID Variables


void correctPosition(int calibrationHeading) {
  // desiredHeading = calibrationHeading * M_PI/180;
  // enablePID = true;
  // turningToPoint = false;
  // timeoutLength = 1000;
  // Brain.resetTimer();
  if (correctingPosition == false) {
      correctingPosition = true; 
  }
  waitUntil(enablePID == false);
  
  float backDistance = (distanceSensorBack.objectDistance(distanceUnits::cm) + 17) / 100;
  float rightDistance = (distanceSensorRight.objectDistance(distanceUnits::cm) + 13) / 100;
  float leftDistance =(distanceSensorLeft.objectDistance(distanceUnits::cm) + 16.8) / 100;

  printf("\n Correcting!");

  if (rightDistance < leftDistance) {
    printf("\n Right < Left!");
    if (calibrationHeading == 0) { //could use currentAbsoluteOrientation
      globalX = backDistance;
      globalY = rightDistance;
    } else if (calibrationHeading == 90) {
      globalX = 3.65 - rightDistance;
      globalY = backDistance;
    } else if (calibrationHeading == 180) {
      globalX = 3.65 - backDistance;
      globalY = 3.65 - rightDistance;
    } else if (calibrationHeading == 270) {
      globalX = rightDistance;
      globalY = 3.65 - backDistance;
    }
  } else {
    printf("\n Left < Right!");
    if (calibrationHeading == 0) { //could use currentAbsoluteOrientation
      globalX = backDistance;
      globalY = 3.65 - leftDistance;
    } else if (calibrationHeading == 90) {
      globalX = leftDistance;
      globalY = backDistance;
    } else if (calibrationHeading == 180) {
      globalX = 3.65 - backDistance;
      globalY = leftDistance;
    } else if (calibrationHeading == 270) {
      globalX = 3.65 - leftDistance;
      globalY = 3.65 - backDistance;
    }
  }

  printf("\n Global X: %f", globalX);
  printf("\n Global Y: %f", globalY);
  correctingPosition = false;
}

void driveTo(float dX, float dY, float dH, float timeoutTime = 2500, float mSpeed = 1.0, bool distanceSensorBool = false) {  // COULD TRY PID WITH VOLTAGE INSTEAD
  desiredX = dX;
  desiredY = dY;
  desiredHeading = dH;
  enablePID = true;
  timeoutLength = timeoutTime;
  Brain.resetTimer();
  maxSpeed = mSpeed;
  useDistance = distanceSensorBool;
}

void driveToAndTurnToPoint(float dX, float dY, float timeoutTime = 2500, float mSpeed = 1.0) {  // COULD TRY PID WITH VOLTAGE INSTEAD
  desiredX = dX;
  desiredY = dY;
  desiredHeading = atan2(dY - globalY, dX - globalX);

  if (desiredHeading < 0) {
    desiredHeading = 2 * M_PI - fabs(desiredHeading);
  }
  enablePID = true;
  turningToPoint = false;
  timeoutLength = timeoutTime;
  Brain.resetTimer();
  maxSpeed = mSpeed;
}

void turnTo(float dH, float timeoutTime = 2500) {
  desiredHeading = dH;
  desiredX = globalX; 
  desiredY = globalY;
  enablePID = true;
  turningToPoint = false;

  timeoutLength = timeoutTime;

  Brain.resetTimer();
}

void turnToPoint(float dX, float dY, float timeoutTime = 2500, bool inverted = false, bool driving = false) {
  desiredTurnX = dX;
  desiredTurnY = dY;
  desiredHeading = atan2(desiredTurnY - globalY, desiredTurnX - globalX);

  if (inverted == true) {
    desiredHeading = desiredHeading - M_PI;
  }

  if (desiredHeading < 0) {
    desiredHeading = 2 * M_PI - fabs(desiredHeading);
  }

  if (driving == false) {
    desiredX = globalX; 
    desiredY = globalY;
  }
  
  enablePID = true;
  turningToPoint = true;
  invertedTurning = inverted;

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
  if(driveError > drivePrevError * 1.1) {
    driveIntegral = 0;
  } 

  driveDerivative = driveError - drivePrevError;

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  //Limit power output to 12V
  if(drivePowerPID > 12) {
    drivePowerPID = 12;
  }

  if(fabs(driveError) < driveMinError) {
    drivePowerPID = 0;
  }

}


void turnPID() {
  if (turningToPoint == true)
  {
    desiredHeading = atan2(desiredTurnY - globalY, desiredTurnX - globalX);
    if (invertedTurning == true) {
      desiredHeading = desiredHeading - M_PI;
    }

    if (desiredHeading < 0) {
      desiredHeading = 2 * M_PI - fabs(desiredHeading);
    }
  }

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

  // reset integral if we pass the target
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

  // if(fabs(turnError) < turnMinError) {
  //   turnPowerPID = 0;
  // }
}

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

      // printf("\n Turn Error %.5f", turnError);
      // printf("\n Turn Integral %.5f", turnIntegral);
      // printf("\n Turn Derivative %.5f", turnDerivative);

      // printf("\n Drive Error %.5f", driveError);
      // printf("\n Drive Integral %.5f", driveIntegral);
      // printf("\n Drive Derivative %.5f", driveDerivative);


      // printf("\n frontLeftPower %f", frontLeftPower);
      // printf("\n frontRightPower %f", frontRightPower);
      // printf("\n backLeftPower %f", backLeftPower);
      // printf("\n backRightPower %f", backRightPower);

      // frontLeft.spin(fwd, xDrivePowerPID + turnPowerPID, velocityUnits::pct); // could use voltage
      // backLeft.spin(fwd, yDrivePowerPID + turnPowerPID , velocityUnits::pct); // could use voltage
      // frontRight.spin(fwd, xDrivePowerPID - turnPowerPID, velocityUnits::pct); // could use voltage
      // backRight.spin(fwd, yDrivePowerPID - turnPowerPID, velocityUnits::pct); // could use voltage


      frontLeft.spin(directionType::fwd, frontLeftPower, voltageUnits::volt);
      frontRight.spin(directionType::fwd, frontRightPower, voltageUnits::volt);
      backLeft.spin(directionType::fwd, backLeftPower, voltageUnits::volt);
      backRight.spin(directionType::fwd, backRightPower, voltageUnits::volt);
   
      if(fabs(driveError) < 0.00254 && fabs(turnError) < 0.003) {
        enablePID = false;
        turningToPoint = false;
        useDistance = false;
        maxSpeed = 1;
      }

      if(Brain.timer(timeUnits::msec) > timeoutLength) {
        enablePID = false;
        turningToPoint = false;
        useDistance = false;
        maxSpeed = 1;
      }

      if (useDistance == true && distanceSensorBack.objectDistance(distanceUnits::cm) < 12) {
        printf("\n object is close!");
        enablePID = false;
        turningToPoint = false;
        useDistance = false;
        maxSpeed = 1;
      }
    }
    //What to do when not using the chassis controls
    else if (userControl == false) {
      
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