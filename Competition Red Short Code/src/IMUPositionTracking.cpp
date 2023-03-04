#include "IMUPositionTracking.h"

#pragma region Odometry
float IMUGlobalX = 0.83;
float IMUGlobalY = 0.33;

// float deltaGlobalX = 0;
// float deltaGlobalY = 0;

// float deltaLocalX = 0;
// float deltaLocalY = 0;

float IMUCurrentAbsoluteOrientation = M_PI/2;
// float prevTheta = M_PI/2;

// float deltaTheta = 0;

float fwdSpeed = 0;
float sdwSpeed = 0;

// float deltaFwdSpeed = 0;
// float deltaSdwSpeed = 0;

// float deltaFwdDist = 0;
// float deltaSdwDist = 0;

// float avgThetaForArc = IMUCurrentAbsoluteOrientation + (deltaTheta / 2);

#pragma endregion Odometry
#pragma endregion Variables

int IMUPositionTracking() {  // COULD TRY PID WITH VOLTAGE INSTEAD
  while (true) {
    IMUCurrentAbsoluteOrientation = (360 - inertialSensor.heading(deg)) * M_PI/180;

    fwdSpeed += inertialSensor.acceleration(xaxis) * sin(IMUCurrentAbsoluteOrientation)  + inertialSensor.acceleration(yaxis) * cos(IMUCurrentAbsoluteOrientation);
    IMUGlobalY += fwdSpeed;
    
    sdwSpeed += inertialSensor.acceleration(yaxis) * sin(IMUCurrentAbsoluteOrientation) + inertialSensor.acceleration(xaxis) * cos(IMUCurrentAbsoluteOrientation);
    IMUGlobalX += sdwSpeed;

    // deltaTheta = IMUCurrentAbsoluteOrientation - prevTheta;

    // prevTheta = IMUCurrentAbsoluteOrientation;
    
    // fwdSpeed += (inertialSensor.acceleration(xaxis) * 0.01); //dist = v * t = a * t * t
    // sdwSpeed += (inertialSensor.acceleration(yaxis) * 0.01);
    
    // deltaFwdDist = fwdSpeed * 0.01;
    // deltaSdwDist = sdwSpeed * 0.01;

    // //If we didn't turn, then we only translated
    // if(deltaTheta == 0) {
    //   deltaLocalX = deltaSdwDist;
    //   // could be either L or R, since if deltaTheta == 0 we assume they're =
    //   deltaLocalY = deltaFwdDist;
    // }
    // //Else, caluclate the new local position
    // else {
    //   //Calculate the changes in the X and Y values (METERS)
    //   //General equation is:
    //   //Distance = 2 * Radius * sin(deltaTheta / 2)
    //   deltaLocalY = 2 * sin(deltaTheta / 2.0) * ((deltaFwdDist / deltaTheta)); // i think this should work idk though
    //   deltaLocalX = 2 * sin(deltaTheta / 2.0) * ((deltaSdwDist / deltaTheta)); 
    // }

    // //The average angle of the robot during it's arc (RADIANS)
    // avgThetaForArc = IMUCurrentAbsoluteOrientation - (deltaTheta / 2);

    // deltaGlobalY = (deltaLocalY * sin(avgThetaForArc)) + (deltaLocalX * cos(avgThetaForArc));
    // deltaGlobalX = (deltaLocalY * cos(avgThetaForArc)) - (deltaLocalX * sin(avgThetaForArc));
    
    // //Wraps angles back around if they ever go under 0 or over 2 pi
    // while(IMUCurrentAbsoluteOrientation >= 2 * M_PI) {
    //   IMUCurrentAbsoluteOrientation -= 2 * M_PI;
    // }
    
    // while(IMUCurrentAbsoluteOrientation < 0) {
    //   IMUCurrentAbsoluteOrientation += 2 * M_PI;
    // }

    // //Update global positions
    // IMUGlobalX += deltaGlobalX;
    // IMUGlobalY += deltaGlobalY;

    // Don't hog cpu
    vex::task::sleep(10);
  }
  return 1;
}
