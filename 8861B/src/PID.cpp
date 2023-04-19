#include "PID.h"

bool fieldOriented = false;
bool correctingPosition = false;

#pragma region PID Varialbes
bool enablePID = true;
bool enableDrivePID = false;
bool resetEncoders = false;
bool turningToPoint = false;

float desiredTurnX = 0;
float desiredTurnY = 0;
float desiredX = 0;
float desiredY = 0;
float desiredHeading = 0;
float timeoutLength = 2500;
float desiredForwardValue = 0;

double turnError = 0;
double turnPrevError = 0;

double turnMinError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.27; //i think the units are radians so no need to convert to radians 

double turnDerivative = 0;

// double turnkP = 10;
// double turnkI = 0.00;
// double turnkD = 0.00;

double turnkP = 7; //8 //1. tune until oscilates //OR only tune this one until it goes close to perfect distance
double turnkI = 1.5; //1.5 //3. tune until it goes perfect distance
double turnkD = 8; //8 //2. tune until it goes under by a little bit
/// T Ultimate = 0.3875s 
/// K Ultimate = 73
double turnPowerPID = 0;

double driveMinError = 0.1;

double driveIntegralBound = 1.5; //converted inches to m (23880E had 1.5 inches)

// double drivekP = 20;
// double drivekI = 0.0;
// double drivekD = 0.0;

double drivekP = 0.5; // 0.5
double drivekI = 0.0; // 0.0? could add it if needed
double drivekD = 0.1; // 0.1

double drivePowerPID = 0;

float driveError; //Desired Value - Sensor Value: Position
float drivePrevError = 0; //Position 20ms ago
float driveDerivative; // error - prevError : Speed
float driveIntegral = 0; // totalError += error : Integral 

float leftPower = 0;
float rightPower = 0;

//distances between robot's current position and the target position
double xDistToTarget = 0;
double yDistToTarget = 0;

//angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

float maxSpeed = 1; //60 max speed good for tank drive 
float maxTurningSpeed = 1;
#pragma endregion PID Variables

void driveFwd(float dFwd, float timeoutTime = 2500, float mSpeed = 1.0) {  // COULD TRY PID WITH VOLTAGE INSTEAD
  desiredForwardValue = dFwd + (forwardRotation.position(degrees) * ((2.75*M_PI)/360));
  desiredHeading = currentAbsoluteOrientation;
  enablePID = true;
  enableDrivePID = true;
  timeoutLength = timeoutTime;
  Brain.resetTimer();
  maxSpeed = mSpeed;
}

void driveTo(float dX, float dY, float timeoutTime = 2500, float mSpeed = 1.0) {  // COULD TRY PID WITH VOLTAGE INSTEAD
  desiredForwardValue = sqrt((pow((dX - globalX),2) + pow((dY - globalY),2))) + (forwardRotation.position(degrees) * ((2.75*M_PI)/360));
  desiredHeading = atan2(dX - globalY, dY - globalX);
  desiredX = dX;
  desiredY = dY;

  desiredTurnX = dX;
  desiredTurnY = dY;
  enablePID = true;
  enableDrivePID = true;
  turningToPoint = true;
  timeoutLength = timeoutTime;
  Brain.resetTimer();
  maxSpeed = mSpeed;
}

void turnTo(float dH, float timeoutTime = 2500) {
  desiredHeading = dH;

  enablePID = true;
  turningToPoint = false;  
  timeoutLength = timeoutTime;

  Brain.resetTimer();
}

void turnToPoint(float dX, float dY, float timeoutTime = 2500) {
  desiredTurnX = dX;
  desiredTurnY = dY;
  desiredHeading = atan2(desiredTurnY - globalY, desiredTurnX - globalX);

  if (desiredHeading < 0) {
    desiredHeading = 2 * M_PI - fabs(desiredHeading);
  }
  
  enablePID = true;
  turningToPoint = true;

  timeoutLength = timeoutTime;

  Brain.resetTimer();
}

void drivePID() {
  
  //Error is equal to the total distance away from the target (uses distance formula with current position and target location)
  // driveError = sqrt(pow((globalX - desiredX), 2) + pow((globalY - desiredY), 2));
  driveError = desiredForwardValue - (forwardRotation.position(degrees) * (2.75*M_PI)/360);
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

  driveDerivative = driveError - drivePrevError; //might need to change the sign or something idk

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  //Limit power output to 12V
  if(drivePowerPID > 12) {
    drivePowerPID = 12;
  }
  if(drivePowerPID < -12) {
    drivePowerPID = -12;
  }

  // if(fabs(driveError) < driveMinError) {
  //   drivePowerPID = 0;
  // }

}

void turnPID() {
  if (turningToPoint == true)
  {
    desiredHeading = atan2(desiredTurnY - globalY, desiredTurnX - globalX);

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
      //get PID values for driving and turning
      if (enableDrivePID == true) {
        drivePID();
      } else {
        drivePowerPID = 0;
      }
      turnPID();

      //set power for each motor
      leftPower = ((drivePowerPID) + turnPowerPID) * maxSpeed;
      rightPower = ((drivePowerPID) - turnPowerPID) * maxSpeed;

      frontLeft.spin(directionType::fwd, leftPower, voltageUnits::volt);
      midLeft.spin(directionType::fwd, leftPower, voltageUnits::volt);
      backLeft.spin(directionType::fwd, leftPower, voltageUnits::volt);
      frontRight.spin(directionType::fwd, rightPower, voltageUnits::volt);
      midRight.spin(directionType::fwd, rightPower, voltageUnits::volt);
      backRight.spin(directionType::fwd, rightPower, voltageUnits::volt);
      
      // if(fabs(turnError) < turnMinError) {
      //   if (enableDrivePID == false) {
      //     enablePID = false;
      //     turningToPoint = false;
      //     maxSpeed = 1;
      //   } else if (fabs(driveError) < driveMinError) {
      //     enablePID = false;
      //     enableDrivePID = false;
      //     turningToPoint = false;
      //     maxSpeed = 1;

      //   }
      // }

      if(Brain.timer(timeUnits::msec) > timeoutLength) {
        enablePID = false;
        enableDrivePID = false;
        turningToPoint = false;
        maxSpeed = 1;
      }
    }
    //What to do when not using the chassis controls
    else {
      frontLeft.stop(brakeType::coast);
      midLeft.stop(brakeType::coast);
      backLeft.stop(brakeType::coast);
      frontRight.stop(brakeType::coast);
      midRight.stop(brakeType::coast);
      backRight.stop(brakeType::coast);
    }
    
    task::sleep(20);

  }

  return 1;
}