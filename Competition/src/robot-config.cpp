#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL = motor(PORT1, ratio18_1, true);
motor FR = motor(PORT10, ratio18_1, false);
motor BL = motor(PORT11, ratio18_1, true);
motor BR = motor(PORT20, ratio18_1, false);
motor MR = motor(PORT19, ratio18_1, false);
motor ML = motor(PORT12, ratio18_1, true);
encoder Left = encoder(Brain.ThreeWirePort.E);
encoder Side = encoder(Brain.ThreeWirePort.G);
inertial Inertial = inertial(PORT18);

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