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
inertial Inertial = inertial(PORT18);
encoder Left = encoder(Brain.ThreeWirePort.G);
motor FlyFront = motor(PORT5, ratio6_1, true);
motor FlyBack = motor(PORT6, ratio6_1, false);
motor Intake = motor(PORT16, ratio18_1, false);
digital_out IndexPiston = digital_out(Brain.ThreeWirePort.C);
digital_out Endgame = digital_out(Brain.ThreeWirePort.B);
motor Indexer = motor(PORT15, ratio18_1, false);
digital_out Pressure = digital_out(Brain.ThreeWirePort.A);
rotation Flywheel = rotation(PORT3, false);
digital_out Angler = digital_out(Brain.ThreeWirePort.D);
rotation Side = rotation(PORT2, false);

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