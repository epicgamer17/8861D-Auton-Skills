#include "vex.h"
// #include "PID.h"
#include "odometry.h" //comment out if you include pid

float PPSPtToPtDistance (float x1, float y1, float x2, float y2);

int PPSSgn (float num);

void PPSTurnToPoint(float dX, float dY);

void PPSTurnPID();

void PPS (float path[][2], float lookAheadDis, int LFIndex);

int PPSTask();