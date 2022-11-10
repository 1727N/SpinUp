#include "vex.h"

extern void FwVelocitySet(int velocityR, float predicted_driveR);

extern bool flywheelSpin;
extern int setVoltage;

int FwControlTask();
int fwTask();