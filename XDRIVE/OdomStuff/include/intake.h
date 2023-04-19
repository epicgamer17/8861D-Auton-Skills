#include "vex.h"
#include "PID.h"

extern bool intakeOn;
extern int discCount;
extern int intakeSpeed;

extern void rollerBlue();

extern void rollerRed();

extern void countDiscs();

extern void toggleIntake();

void visionTurnPID();

int visionPIDTask();

extern void visionPickUpDisc();