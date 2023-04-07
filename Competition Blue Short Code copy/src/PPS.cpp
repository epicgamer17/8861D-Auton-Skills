#include "PPS.h"

////////////////////////////
/// Pure Pursuit 
////////////////////////////

int lastFoundIndex = 0;

float PPSPtToPtDistance (float x1, float y1, float x2, float y2) {
    float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
    return dist;
  }

// returns -1 if num is negative, 1 otherwise
int PPSSgn (float num) {
  if (num >= 0)
    return 1;
  else
    return -1;
  }

#include <bits/stdc++.h>
using namespace std;


#pragma region PID Varialbes

float PPSDesiredTurnX = 0;
float PPSDesiredTurnY = 0;
float PPSDesiredHeading = 0;

double PPSTurnError = 0;
// double turnPrevError = 0;

// double turnMinError = 0.01;

// double turnIntegral = 0;
// double turnIntegralBound = 0.09; //i think the units are radians so no need to convert to radians 

// double turnDerivative = 0;

double PPSTurnkP = 13.00;
// double turnkI = 1.00;
// double turnkD = 10.00;

double PPSTurnPowerPID = 0;

// double frontLeftPower = 0;
// double frontRightPower = 0;
// double backLeftPower = 0;
// double backRightPower = 0;

double PPSLinearVelocity = 7;

#pragma endregion PID Variables

void PPSTurnToPoint(float dX, float dY) {
  PPSDesiredTurnX = dX;
  PPSDesiredTurnY = dY;
  PPSDesiredHeading = atan2(PPSDesiredTurnY - globalY, PPSDesiredTurnX - globalX);

  if (PPSDesiredHeading < 0) {
    PPSDesiredHeading = 2 * M_PI - fabs(PPSDesiredHeading);
  }
  
  Brain.resetTimer();
}

void PPSTurnPID() {

  PPSDesiredHeading = atan2(PPSDesiredTurnY - globalY, PPSDesiredTurnX - globalX);

  if (PPSDesiredHeading < 0) {
    PPSDesiredHeading = 2 * M_PI - fabs(PPSDesiredHeading);
  }

  //Error is equal to the difference between the current facing direction and the target direction
  PPSTurnError = currentAbsoluteOrientation - PPSDesiredHeading;

  if(fabs(PPSTurnError) > M_PI) {
    PPSTurnError = (PPSTurnError/fabs(PPSTurnError)) * -1 * fabs(2 * M_PI - PPSTurnError);
  }

  //only use integral if close enough to target
  // if(fabs(turnError) < turnIntegralBound) {
  //   turnIntegral += turnError;
  // }
  // else {
  //   turnIntegral = 0;
  // }

  //reset integral if we pass the target
  // if(turnError * turnPrevError < 0) {
  //   turnIntegral = 0;
  // } 

  // turnDerivative = turnError - turnPrevError;

  // turnPrevError = turnError;

  PPSTurnPowerPID = (PPSTurnError * PPSTurnkP /* + turnIntegral * turnkI + turnDerivative * turnkD */);

  //Limit power output to 12V
  if(PPSTurnPowerPID > 12) {
    PPSTurnPowerPID = 12;
  }

  // if(fabs(PPSTurnError) < PPSTurnMinError) {
  //   PPSTurnPowerPID = 0;
  // }
}

void PPS(vector<vector<float>> path, float lookAheadDis, int LFIndex) {
  // # use for loop to search intersections
  bool intersectFound = false;
  int startingIndex = LFIndex;
  float tempX = globalX;
  float tempY = globalY;

  for (int i = startingIndex; i < (path.size() - 1); i ++) {
    // # beginning of line-circle intersection code
    float x1 = path[i][0] - globalX;
    float y1 = path[i][1] - globalY;
    float x2 = path[i+1][0] - globalX;
    float y2 = path[i+1][1] - globalY;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dr = sqrt(pow(dx,2) + pow(dy,2));
    float D = x1*y2 - x2*y1;
    float discriminant = (pow(lookAheadDis,2)) * pow(dr,2) - pow(D,2);


    if (discriminant >= 0) {
      float sol_x1 = (D * dy + PPSSgn(dy) * dx * sqrt(discriminant)) / pow(dr,2);
      float sol_x2 = (D * dy - PPSSgn(dy) * dx * sqrt(discriminant)) / pow(dr,2);
      float sol_y1 = (- D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr,2);
      float sol_y2 = (- D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr,2);

      float goalX1 = sol_x1 + globalX;
      float goalY1 = sol_y1 + globalY;
      float goalX2 = sol_x2 + globalX;
      float goalY2 = sol_y2 + globalY;
      // # end of line-circle intersection code

      float minX = fmin(path[i][0], path[i+1][0]);
      float minY = fmin(path[i][1], path[i+1][1]);
      float maxX = fmax(path[i][0], path[i+1][0]);
      float maxY = fmax(path[i][1], path[i+1][1]);

      // # if one or both of the solutions are in range
      if (((minX <= goalX1 <= maxX) && (minY <= goalY1 <= maxY)) || ((minX <= goalX2 <= maxX) && (minY <= goalY2 <= maxY))) {
        intersectFound = true;

        // # if both solutions are in range, check which one is better
        if (((minX <= goalX1 <= maxX) and (minY <= goalY1 <= maxY)) and ((minX <= goalX2 <= maxX) and (minY <= goalY2 <= maxY))) {
          // # make the decision by compare the distance between the intersections and the next point in path
          printf("both solutions are in range take the closest one \n");
          if (PPSPtToPtDistance(goalX1, goalY1, path[i+1][0], path[i+1][1]) < PPSPtToPtDistance(goalX2, goalY2, path[i+1][0], path[i+1][1])) {
            tempX = goalX1;
            tempY = goalY1;
          } else {
            tempX = goalX2;
            tempY = goalY2;
          }
        }
        // # if not both solutions are in range, take the one that's in range
        else {
          // # if solution pt1 is in range, set that as goal point
          printf("only one solution in range \n");
          if ((minX <= goalX1 <= maxX) and (minY <= goalY1 <= maxY)) {
            tempX = goalX1;
            tempY = goalY1;
          } else {
            tempX = goalX2;
            tempY = goalY2;
          }
        }  
        // # only exit loop if the solution pt found is closer to the next pt in path than the current pos
        if (PPSPtToPtDistance(tempX, tempY, path[i+1][0], path[i+1][1]) < PPSPtToPtDistance(globalX, globalY, path[i+1][0],path[i+1][1])) {
          // # update lastFoundIndex and exit
          printf("Exit the loop because the sol point was closer to the next pt in the path than the current position \n");
          LFIndex = i;
          break;
        } else {
          // # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
          printf("in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward \n");
          LFIndex = i+1;
        }
      }  
      // # if no solutions are in range
      else {
        intersectFound = false;
        // # no new intersection found, potentially deviated from the path
        printf("no new intersection found, potentially deviated from the path \n");
        tempX = path[LFIndex][0];
        tempY = path[LFIndex][1];
      }
    }
  }

  // driveToAndTurnToPoint(tempX, tempY, 10000, 1); //for odom pid

  
  PPSTurnToPoint(tempX, tempY);

  lastFoundIndex = LFIndex; 
  return;
}


int PPSTask() {
// vector<vector<float>> path1 {{0, 0}, {0.571194595265405, -0.4277145118491421}, {1.1417537280142898, -0.8531042347260006}, {1.7098876452457967, -1.2696346390611464}, {2.2705328851607995, -1.6588899151216996}, {2.8121159420106827, -1.9791445882187304}, {3.314589274316711, -2.159795566252656}, {3.7538316863009027, -2.1224619985315876}, {4.112485112342358, -1.8323249172947023}, {4.383456805594431, -1.3292669972090994}, {4.557386228943757, -0.6928302521681386}, {4.617455513800438, 0.00274597627737883}, {4.55408382321606, 0.6984486966257434}, {4.376054025556597, 1.3330664239172116}, {4.096280073621794, 1.827159263675668}, {3.719737492364894, 2.097949296701878}, {3.25277928312066, 2.108933125822431}, {2.7154386886417314, 1.9004760368018616}, {2.1347012144725985, 1.552342808106984}, {1.5324590525923942, 1.134035376721349}, {0.9214084611203568, 0.6867933269918683}, {0.30732366808208345, 0.22955002391894264}, {-0.3075127599907512, -0.2301742560363831}, {-0.9218413719658775, -0.6882173194028102}, {-1.5334674079795052, -1.1373288016589413}, {-2.1365993767877467, -1.5584414896876835}, {-2.7180981380280307, -1.9086314914221845}, {-3.2552809639439704, -2.1153141204181285}, {-3.721102967810494, -2.0979137913841046}, {-4.096907306768644, -1.8206318841755131}, {-4.377088212533404, -1.324440752295139}, {-4.555249804461285, -0.6910016662308593}, {-4.617336323713965, 0.003734984720118972}, {-4.555948690867849, 0.7001491248072772}, {-4.382109193278264, 1.3376838311365633}, {-4.111620918085742, 1.8386823176628544}, {-3.7524648889185794, 2.1224985058331005}, {-3.3123191098095615, 2.153588702898333}, {-2.80975246649598, 1.9712114570096653}, {-2.268856462266256, 1.652958931009528}, {-1.709001159778989, 1.2664395490411673}, {-1.1413833971013372, 0.8517589252820573}, {-0.5710732645795573, 0.4272721367616211}, {0, 0}, {0.571194595265405, -0.4277145118491421}};
vector<vector<float>> path1 {{0, 0}, {1.2, 1.2}, {0,1.2}, {0, 0}};
  while (true) {
    if (lastFoundIndex >= path1.size()) {
      lastFoundIndex = 0;
    }
    PPS(path1, 0.4, lastFoundIndex);
    
    //get PID values for driving and turning
    PPSTurnPID();

    //set power for each motor
    // frontLeftPower = ((linearVelocity /* if you includ the setDrivePower then you could drive without needing to face a certain direction*/) + turnPowerPID);
    // frontRightPower = ((linearVelocity) - turnPowerPID);
    // backLeftPower = ((linearVelocity) + turnPowerPID);
    // backRightPower = ((linearVelocity) - turnPowerPID);

    frontLeft.spin(directionType::fwd, ((PPSLinearVelocity) + PPSTurnPowerPID), voltageUnits::volt);
    frontRight.spin(directionType::fwd, ((PPSLinearVelocity) - PPSTurnPowerPID), voltageUnits::volt);
    backLeft.spin(directionType::fwd, ((PPSLinearVelocity) + PPSTurnPowerPID), voltageUnits::volt);
    backRight.spin(directionType::fwd, ((PPSLinearVelocity) - PPSTurnPowerPID), voltageUnits::volt);
    //What to do when not using the chassis controls    
    
    task::sleep(20);
  }
  return 1;
}
