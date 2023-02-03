#include "PPS.h"

////////////////////////////
/// Pure Pursuit 
////////////////////////////

int lastFoundIndex = 0;
// float lookAheadDis = 8;

// this determines how long (how many frames) the animation will run. 400 frames takes around 30 seconds.

// float ptToPtDistance (float x1, float y1, float x2, float y2) {
//     float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
//     return dist;
//   }

// // returns -1 if num is negative, 1 otherwise
// int sgn (float num) {
//   if (num >= 0)
//     return 1;
//   else
//     return -1;
//   }

// # this function needs to return 3 things IN ORDER: goalPt, lastFoundIndex, turnVel
// # think about this function as a snapshot in a while loop
// # given all information about the robot's current state, what should be the goalPt, lastFoundIndex, and turnVel?
// # the LFindex takes in the value of lastFoundIndex as input. Looking at it now I can't remember why I have it.
// # it is this way because I don't want the global lastFoundIndex to get modified in this function, instead, this function returns the updated lastFoundIndex value 
// # this function will be feed into another function for creating animation
#include <bits/stdc++.h>
using namespace std;

void PPS(vector<vector<float>> path, float lookAheadDis, int LFIndex) {
  printf("%d", LFIndex);
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
      float sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / pow(dr,2);
      float sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / pow(dr,2);
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

      // printf("\n Goal X1 %f", goalX1);
      // printf("\n Goal Y1 %f", goalY1);
      // printf("\n Goal X2 %f", goalX2);
      // printf("\n Goal Y2 %f", goalY2);
      
      // printf("\n Temp X %f", tempX);
      // printf("\n Temp Y %f", tempY);

      printf("\n Last Index %d", LFIndex);

      // # if one or both of the solutions are in range
      if (((minX <= goalX1 <= maxX) && (minY <= goalY1 <= maxY)) || ((minX <= goalX2 <= maxX) && (minY <= goalY2 <= maxY))) {
        intersectFound = true;

        // # if both solutions are in range, check which one is better
        if (((minX <= goalX1 <= maxX) and (minY <= goalY1 <= maxY)) and ((minX <= goalX2 <= maxX) and (minY <= goalY2 <= maxY))) {
          // # make the decision by compare the distance between the intersections and the next point in path
          printf("both solutions are in range take the closest one \n");
          if (ptToPtDistance(goalX1, goalY1, path[i+1][0], path[i+1][1]) < ptToPtDistance(goalX2, goalY2, path[i+1][0], path[i+1][1])) {
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
        if (ptToPtDistance(tempX, tempY, path[i+1][0], path[i+1][1]) < ptToPtDistance(globalX, globalY, path[i+1][0],path[i+1][1])) {
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
        // # follow path[lastFoundIndex]
        printf("no new intersection found, potentially deviated from the path \n");
        tempX = path[LFIndex][0];
        tempY = path[LFIndex][1];
      }
    }
  }
  // # obtained goal point, now compute turn vel
  // # initialize proportional controller constant
  // Kp = 3

  // # calculate absTargetAngle with the atan2 function
  // absTargetAngle = math.atan2 (goalPt[1]-currentPos[1], goalPt[0]-currentPos[0]) *180/pi
  // if absTargetAngle < 0: absTargetAngle += 360

  // # compute turn error by finding the minimum angle
  // turnError = absTargetAngle - currentHeading
  // if turnError > 180 or turnError < -180 :
  //   turnError = -1 * sgn(turnError) * (360 - abs(turnError))
  
  // # apply proportional controller
  // turnVel = Kp*turnError
  

  driveToAndTurnToPoint(tempX, tempY, 10000, 1);
  lastFoundIndex = LFIndex; 
  return;
  // return goalPt, lastFoundIndex, turnVel
}


int PPSTask() {
// vector<vector<float>> path1 {{0, 0}, {0.571194595265405, -0.4277145118491421}, {1.1417537280142898, -0.8531042347260006}, {1.7098876452457967, -1.2696346390611464}, {2.2705328851607995, -1.6588899151216996}, {2.8121159420106827, -1.9791445882187304}, {3.314589274316711, -2.159795566252656}, {3.7538316863009027, -2.1224619985315876}, {4.112485112342358, -1.8323249172947023}, {4.383456805594431, -1.3292669972090994}, {4.557386228943757, -0.6928302521681386}, {4.617455513800438, 0.00274597627737883}, {4.55408382321606, 0.6984486966257434}, {4.376054025556597, 1.3330664239172116}, {4.096280073621794, 1.827159263675668}, {3.719737492364894, 2.097949296701878}, {3.25277928312066, 2.108933125822431}, {2.7154386886417314, 1.9004760368018616}, {2.1347012144725985, 1.552342808106984}, {1.5324590525923942, 1.134035376721349}, {0.9214084611203568, 0.6867933269918683}, {0.30732366808208345, 0.22955002391894264}, {-0.3075127599907512, -0.2301742560363831}, {-0.9218413719658775, -0.6882173194028102}, {-1.5334674079795052, -1.1373288016589413}, {-2.1365993767877467, -1.5584414896876835}, {-2.7180981380280307, -1.9086314914221845}, {-3.2552809639439704, -2.1153141204181285}, {-3.721102967810494, -2.0979137913841046}, {-4.096907306768644, -1.8206318841755131}, {-4.377088212533404, -1.324440752295139}, {-4.555249804461285, -0.6910016662308593}, {-4.617336323713965, 0.003734984720118972}, {-4.555948690867849, 0.7001491248072772}, {-4.382109193278264, 1.3376838311365633}, {-4.111620918085742, 1.8386823176628544}, {-3.7524648889185794, 2.1224985058331005}, {-3.3123191098095615, 2.153588702898333}, {-2.80975246649598, 1.9712114570096653}, {-2.268856462266256, 1.652958931009528}, {-1.709001159778989, 1.2664395490411673}, {-1.1413833971013372, 0.8517589252820573}, {-0.5710732645795573, 0.4272721367616211}, {0, 0}, {0.571194595265405, -0.4277145118491421}};
vector<vector<float>> path1 {{0, 0}, {1.2, 1.2}, {0,1.2}, {0, 0}};
  while (true) {
    if (lastFoundIndex >= path1.size()) {
      lastFoundIndex = 0;
    }
    printf("%d", lastFoundIndex);
    PPS(path1, 0.4, lastFoundIndex);
  }
  return 1;
}