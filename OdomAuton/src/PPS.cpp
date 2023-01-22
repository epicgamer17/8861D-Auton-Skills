#include "PPS.h"
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

float ptToPtDistance (float x1, float y1, float x2, float y2) {
    float dist = sqrt((pow((x2 - x1),2) + pow((y2 - y1),2)));
    return dist;
  }

// returns -1 if num is negative, 1 otherwise
int sgn (float num) {
  if (num >= 0)
    return 1;
  else
    return -1;
  }

void PPS (float path[][2], float lookAheadDis){
    // use for loop to search intersections
    bool intersectFound;
    int startingIndex = lastFoundIndex;
    float tempX = 0;
    float tempY = 0;

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

            intersectFound = true;

            // if both solutions are in range, check which one is better
            if (((minX <= solPt1[0] && solPt1[0] <= maxX) && (minY <= solPt1[1] && solPt1[1] <= maxY)) && ((minX <= solPt2[0] && solPt2[0] <= maxX) && (minY <= solPt2[1] && solPt2[1] <= maxY))) {
              // make the decision by compare the distance between the intersections and the next point in path
              printf("\n Found Solution Both in Range");
              if (ptToPtDistance(solPt1[0], solPt1[1], path[i+1][0], path[i+1][1]) < ptToPtDistance(solPt2[0], solPt2[1],  path[i+1][0], path[i+1][1])) {
                  tempX = solPt1[0];
                  tempY = solPt1[1];
              }
              else {
                  tempX = solPt2[0];
                  tempY = solPt2[1];
              }
            }
            // if not both solutions are in range, take the one that's in range
            else {
              // if solution pt1 is in range, set that as goal point
              printf("\n Found Solution One in Range");
              if ((minX <= solPt1[0] && solPt1[0] <= maxX) && (minY <= solPt1[1] && solPt1[1] <= maxY)) {
                  tempX = solPt1[0];
                  tempY = solPt1[1];
              }
              else {
                tempX = solPt2[0];
                tempY = solPt2[1];
              }
            }
            // only exit loop if the solution pt found is closer to the next pt in path than the current pos
            if (ptToPtDistance(tempX, tempY,  path[i+1][0], path[i+1][1]) < ptToPtDistance(globalX, globalY,  path[i+1][0], path[i+1][1])) {
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
              intersectFound = false;
              // no new intersection found, potentially deviated from the path
              // follow path[lastFoundIndex]
              tempX = path[lastFoundIndex][0];
              tempY = path[lastFoundIndex][1];
          }
        // if determinant < 0
        } else { 
          intersectFound = false;
          // no new intersection found, potentially deviated from the path
          // follow path[lastFoundIndex]
          tempX = path[lastFoundIndex][0];
          tempY = path[lastFoundIndex][1];
          printf("\n No Solution");

        }
    
    //   # apply proportional controller
    driveToAndTurnToPoint(tempX, tempY, 2500, 1);
    printf("\n X %f", tempX);
    printf("\n Y %f", tempY);
    printf("\n Sol X %f", solPt1[0]);
    printf("\n Sol Y %f", solPt1[1]);
    printf("\n Global X %f", globalX);
    printf("\n Global Y %f", globalY);
    printf("\n Intersect Found %d", intersectFound);

    printf("\n Last Found Index %d", lastFoundIndex);
    printf("\n Size of Path %d", sizeof(path[0][0]));

    if (startingIndex == sizeof(*path)) {
        if (fabs(dx) < 1 && fabs(dy) < 1) {
            lastFoundIndex = i+1;
        }  
    }
    task::sleep(200);
  }
}

int PPSTask() {
float path1[][2] = {{0.571194595265405, -0.4277145118491421}, {1.1417537280142898, -0.8531042347260006}, {1.7098876452457967, -1.2696346390611464}, {2.2705328851607995, -1.6588899151216996}, {2.8121159420106827, -1.9791445882187304}, {3.314589274316711, -2.159795566252656}, {3.7538316863009027, -2.1224619985315876}, {4.112485112342358, -1.8323249172947023}, {4.383456805594431, -1.3292669972090994}, {4.557386228943757, -0.6928302521681386}, {4.617455513800438, 0.00274597627737883}, {4.55408382321606, 0.6984486966257434}, {4.376054025556597, 1.3330664239172116}, {4.096280073621794, 1.827159263675668}, {3.719737492364894, 2.097949296701878}, {3.25277928312066, 2.108933125822431}, {2.7154386886417314, 1.9004760368018616}, {2.1347012144725985, 1.552342808106984}, {1.5324590525923942, 1.134035376721349}, {0.9214084611203568, 0.6867933269918683}, {0.30732366808208345, 0.22955002391894264}, {-0.3075127599907512, -0.2301742560363831}, {-0.9218413719658775, -0.6882173194028102}, {-1.5334674079795052, -1.1373288016589413}, {-2.1365993767877467, -1.5584414896876835}, {-2.7180981380280307, -1.9086314914221845}, {-3.2552809639439704, -2.1153141204181285}, {-3.721102967810494, -2.0979137913841046}, {-4.096907306768644, -1.8206318841755131}, {-4.377088212533404, -1.324440752295139}, {-4.555249804461285, -0.6910016662308593}, {-4.617336323713965, 0.003734984720118972}, {-4.555948690867849, 0.7001491248072772}, {-4.382109193278264, 1.3376838311365633}, {-4.111620918085742, 1.8386823176628544}, {-3.7524648889185794, 2.1224985058331005}, {-3.3123191098095615, 2.153588702898333}, {-2.80975246649598, 1.9712114570096653}, {-2.268856462266256, 1.652958931009528}, {-1.709001159778989, 1.2664395490411673}, {-1.1413833971013372, 0.8517589252820573}, {-0.5710732645795573, 0.4272721367616211}, {0, 0}, {0.571194595265405, -0.4277145118491421}};
  while (true) {
      PPS(path1, 10);
  }
  return 1;
}