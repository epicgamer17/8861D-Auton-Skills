#include "vex.h"
#include "PID.h"


float ptToPtDistance (float x1, float y1, float x2, float y2);

int sgn (float num);

void PPS (float path[][2], float lookAheadDis);

int PPSTask();