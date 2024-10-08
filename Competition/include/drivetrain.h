#include "vex.h"
#include "odometry.h"

extern double xTargetLocation;
extern double yTargetLocation;
extern double targetFacingAngle;

extern double turnkP;
extern double turnkI;
extern double turnkD;

extern double drivekP;
extern double drivekI;
extern double drivekD;

extern bool runChassisControl;

extern void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength, double maxSpeed);

extern void directDrive(double xDist, double timeOutLength, double maxSpeed);

extern void turnTo(double targetAngle, double timeOutLength);

extern void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength, double maxSpeed);

void drivePID();
void turnPID();

int chassisControl();
