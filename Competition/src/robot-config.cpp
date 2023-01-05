#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL = motor(PORT1, ratio18_1, true);
motor FR = motor(PORT9, ratio18_1, false);
motor BL = motor(PORT11, ratio18_1, true);
motor BR = motor(PORT20, ratio18_1, false);
inertial Inertial = inertial(PORT17);
encoder Left = encoder(Brain.ThreeWirePort.G);
encoder Side = encoder(Brain.ThreeWirePort.E);
motor FlyFront = motor(PORT5, ratio6_1, true);
motor FlyBack = motor(PORT6, ratio6_1, false);
/*vex-vision-config:begin*/
vision Vision = vision (PORT3, 50);
/*vex-vision-config:end*/
motor Intake = motor(PORT12, ratio18_1, true);
digital_out Puncher = digital_out(Brain.ThreeWirePort.A);
digital_out Catapult = digital_out(Brain.ThreeWirePort.B);
encoder Right = encoder(Brain.ThreeWirePort.C);
motor Roller = motor(PORT7, ratio18_1, false);
motor Indexer = motor(PORT4, ratio18_1, false);

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