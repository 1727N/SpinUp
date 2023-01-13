#include "vex.h"

extern void FwVelocitySet(int velocityR, float predicted_driveR);

extern bool flywheelSpin;
extern int setVoltage;
extern bool DRIVER_CONTROL;
extern bool flyWheelOn;

int FwControlTask();
int fwTask();

extern double FW_MAX_POWER;