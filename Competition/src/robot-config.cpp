#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL = motor(PORT11, ratio18_1, true);
motor FR = motor(PORT1, ratio18_1, false);
motor BL = motor(PORT19, ratio18_1, false);
motor BR = motor(PORT9, ratio18_1, true);
inertial Inertial = inertial(PORT17);
encoder Left = encoder(Brain.ThreeWirePort.G);
encoder Right = encoder(Brain.ThreeWirePort.A);
encoder Side = encoder(Brain.ThreeWirePort.E);
motor FlyFront = motor(PORT5, ratio6_1, true);
motor FlyBack = motor(PORT6, ratio6_1, false);
/*vex-vision-config:begin*/
vision Vision = vision (PORT3, 50);
/*vex-vision-config:end*/
motor Intake = motor(PORT12, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}