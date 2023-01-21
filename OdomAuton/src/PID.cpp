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

////////////////////////////
/// Pure Pursuit 
////////////////////////////

int lastFoundIndex = 0;
float lookAheadDis = 0.8;
float linearVel = 100;

// set this to true if you use rotations
bool using_rotation = false;

// this determines how long (how many frames) the animation will run. 400 frames takes around 30 seconds.
int numOfFrames = 400;

float goalPt[] = {0, 0};

float pt_to_pt_distance (float x1, float y1, float x2, float y2) {
    float dist = sqrt(( pow((x2 - x1),2) + pow((y2 - y1),2)));
    return dist;
  }

// returns -1 if num is negative, 1 otherwise
int sgn (float num) {
  if (num >= 0)
    return 1;
  else
    return -1;
  }

void pure_pursuit_step (float path[][2], float currentHeading, float lookAheadDis, int LFindex, int* lastINDEX){
    // use for loop to search intersections
    int lastFoundIndex = LFindex;
    bool intersectFound;
    int startingIndex = lastFoundIndex;

    for (int i = startingIndex; i < (sizeof(*path)); i++) {
        // beginning of line-circle intersection code
        float x1 = path[i][0] - globalX;
        float y1 = path[i][1] - globalY;
        float x2 = path[i+1][0] - globalX;
        float y2 = path[i+1][1] - globalY;
        float dx = x2 - x1;
        float dy = y2 - y1;
        float dr = sqrt(pow(dx,2) + pow(dy,2));
        float D = x1*y2 - x2*y1;
        float discriminant = (pow(lookAheadDis,2)) * (pow(dr,2)) - pow(D,2);
        float solX1;
        float solX2;
        float solY1;
        float solY2;
        float solPt1[2];
        float solPt2[2];
        float minX;
        float minY;
        float maxX;
        float maxY;

        if (discriminant >= 0) {
            solX1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / pow(dr,2);
            solX2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / pow(dr,2);
            solY1 = (- D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr,2);
            solY2 = (- D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr,2);

            solPt1[0] = {solX1 + globalX};
            solPt2[0] = {solX2 + globalX};
            solPt1[1] = {solY1 + globalY};
            solPt2[1] = {solY2 + globalY};
            // end of line-circle intersection code

            minX = fmin(path[i][0], path[i+1][0]);
            minY = fmin(path[i][1], path[i+1][1]);
            maxX = fmax(path[i][0], path[i+1][0]);
            maxY = fmax(path[i][1], path[i+1][1]);

        // if one or both of the solutions are in range
        if (((minX <= solPt1[0] <= maxX) && (minY <= solPt1[1] <= maxY)) || ((minX <= solPt2[0] <= maxX) && (minY <= solPt2[1] <= maxY))) {

            intersectFound = 1;

            // if both solutions are in range, check which one is better
            if (((minX <= solPt1[0] && solPt1[0] <= maxX) && (minY <= solPt1[1] && solPt1[1] <= maxY)) && ((minX <= solPt2[0] && solPt2[0] <= maxX) && (minY <= sol_pt2[1] && sol_pt2[1] <= maxY))) {
            // make the decision by compare the distance between the intersections and the next point in path
            if (pt_to_pt_distance(solPt1[0], solPt1[1], path[i+1][0], path[i+1][1]) < pt_to_pt_distance(solPt2[0], solPt2[1],  path[i+1][0], path[i+1][1])) {
                desiredX = solPt1[0];
                desiredY = solPt1[1];
            }
            else {
                desiredX = solPt2[0];
                desiredY = solPt2[1];
            }
            }
            // if not both solutions are in range, take the one that's in range
            else {
            // if solution pt1 is in range, set that as goal point
            if ((minX <= solPt1[0] && solPt1[0] <= maxX) && (minY <= solPt1[1] && solPt1[1] <= maxY)) {
                desiredX = solPt1[0];
                desiredY = solPt1[1];
            }
            else {
                desiredX = solPt2[0];
                desiredY = solPt2[1];
                }
            }
            // only exit loop if the solution pt found is closer to the next pt in path than the current pos
            if (pt_to_pt_distance(desiredX, desiredY,  path[i+1][0], path[i+1][1]) < pt_to_pt_distance(globalX, globalY,  path[i+1][0], path[i+1][1])) {
                // update lastFoundIndex and exit
                lastFoundIndex = i;
                break;
            }
            else {
                // in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                lastFoundIndex = i+1;
            }
        }   
        // if no solutions are in range
        else {
            intersectFound = 0;
            // no new intersection found, potentially deviated from the path
            // follow path[lastFoundIndex]
            desiredX = path[lastFoundIndex][0];
            desiredY = path[lastFoundIndex][1];
        }
        // if determinant < 0
        }
        else { 
            intersectFound = 0;
            // no new intersection found, potentially deviated from the path
            // follow path[lastFoundIndex]
            desiredX = path[lastFoundIndex][0];
            desiredY = path[lastFoundIndex][1];
        }
    
    //   # apply proportional controller
    *lastINDEX = lastFoundIndex;
    if (startingIndex == sizeof(*path)) {
        if (fabs(dx) < 1 && fabs(dy) < 1) {
            lastFoundIndex = i+1;
        }  
    }
    task::sleep(200);
  }
}