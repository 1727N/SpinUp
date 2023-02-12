#include "vex.h"

extern void FwVelocitySet(int velocityR, float predicted_driveR);

extern bool flywheelSpin;
extern int setVoltage;
extern bool DRIVER_CONTROL;
extern bool flyWheelOn;

extern double flykP;
extern double flykI;
extern double flykD;

int FwControlTask();
int fwTask();

extern double FW_MAX_POWER;
extern double flywheelVoltage;