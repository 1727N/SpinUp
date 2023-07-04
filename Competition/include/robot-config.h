using namespace vex;

extern brain Brain;

// VEXcode devices

extern controller Controller1;
extern motor FL;
extern motor FR;
extern motor ML;
extern motor MR;
extern motor BL;
extern motor BR;

extern encoder Left;
extern encoder Side;
extern inertial Inertial;

extern motor FlyOne;
extern motor FlyTwo;
extern motor FlyThree;
extern motor Indexer;

extern limit Trigger;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );