using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FL;
extern motor FR;
extern motor BL;
extern motor BR;
extern inertial Inertial;
extern encoder Left;
extern encoder Right;
extern encoder Side;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );