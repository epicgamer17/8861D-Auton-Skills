#include "odometry.h"

float trackingWheelRadius = 1.375;
#pragma region Odometry
float globalX = 144 - 17;  //start position in inches
float globalY = 144 - 57;  //start position in inches

float deltaGlobalX = 0;
float deltaGlobalY = 0;

float deltaLocalX = 0;
float deltaLocalY = 0;

float currentAbsoluteOrientation = 3*M_PI/2; //set start heading
float prevTheta = 3*M_PI/2; //set start heading

float deltaTheta = 0;

float avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

float fwdTrackingPos = 0;
float sdwTrackingPos = 0;

float prevFwdRotationPos = 0;
float prevSdwRotationPos = 0;

float deltaFwdDist = 0;
float deltaSdwDist = 0;

float totalDeltaFwdPos = 0;
float totalDeltaSdwPos = 0;
#pragma endregion Odometry
#pragma endregion Variables

int positionTracking() {  // COULD TRY PID WITH VOLTAGE INSTEAD
  while (true) {
    currentAbsoluteOrientation = (360 - inertialSensor.heading(deg)) * M_PI/180;

    deltaTheta = currentAbsoluteOrientation - prevTheta;

    prevTheta = currentAbsoluteOrientation;
    
    // Get motor positions
    fwdTrackingPos = forwardRotation.position(deg);
    sdwTrackingPos = sidewaysRotation.position(deg);

    deltaFwdDist = ((fwdTrackingPos - prevFwdRotationPos) * M_PI/180) * trackingWheelRadius;
    deltaSdwDist = ((sdwTrackingPos - prevSdwRotationPos) * M_PI/180) * trackingWheelRadius;


    prevFwdRotationPos = fwdTrackingPos;
    prevSdwRotationPos = sdwTrackingPos;

    totalDeltaFwdPos += deltaFwdDist;
    totalDeltaSdwPos += deltaSdwDist;
    
    //If we didn't turn, then we only translated
    if(deltaTheta == 0) {
      deltaLocalX = deltaSdwDist;
      // could be either L or R, since if deltaTheta == 0 we assume they're =
      deltaLocalY = deltaFwdDist;
    }
    //Else, caluclate the new local position
    else {
      //Calculate the changes in the X and Y values (METERS)
      //General equation is:
      //Distance = 2 * Radius * sin(deltaTheta / 2)
      deltaLocalY = 2 * sin(deltaTheta / 2.0) * ((deltaFwdDist / deltaTheta) + 2.2); //-1.29 for 180 degree turn // -1.50 for 90 degree turn //Measure Forward Tracking wheel Track Radius
      deltaLocalX = 2 * sin(deltaTheta / 2.0) * ((deltaSdwDist / deltaTheta) + 5.5); //-4.14 measured for 360 degree turn // -3.89 measured for 90 degree turn //check if deltaSdwdist is negative when deltaTheta is positive, if so use + otherwise use - // before was + 0.065 for best results //Measure Sideways Track Radius
      // Sideways motor value changes 3170 over 5 turns
      // forward value changes 7 over 5 turns 
    }

    //The average angle of the robot during it's arc (RADIANS)
    avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

    deltaGlobalY = (deltaLocalY * sin(avgThetaForArc)) + (deltaLocalX * cos(avgThetaForArc));
    deltaGlobalX = (deltaLocalY * cos(avgThetaForArc)) - (deltaLocalX * sin(avgThetaForArc));
    
    //Wraps angles back around if they ever go under 0 or over 2 pi
    while(currentAbsoluteOrientation >= 2 * M_PI) {
      currentAbsoluteOrientation -= 2 * M_PI;
    }
    
    while(currentAbsoluteOrientation < 0) {
      currentAbsoluteOrientation += 2 * M_PI;
    }

    //Update global positions
    globalX += deltaGlobalX;
    globalY += deltaGlobalY;

    // Don't hog cpu
    vex::task::sleep(10);
  }
  return 1;
}
