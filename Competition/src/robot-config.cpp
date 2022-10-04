#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontLeftDrive = motor(PORT11, ratio18_1, true);
motor FrontRightDrive = motor(PORT1, ratio18_1, false);
motor BackLeftDrive = motor(PORT19, ratio18_1, false);
motor BackRightDrive = motor(PORT9, ratio18_1, true);
rotation STrack = rotation(PORT20, true);
inertial Inertial6 = inertial(PORT5);
encoder Left = encoder(Brain.ThreeWirePort.G);
encoder Right = encoder(Brain.ThreeWirePort.A);

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