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
extern motor FlyFront;
extern motor FlyBack;
extern motor Intake;
extern digital_out IndexPiston;
extern digital_out Endgame;
extern motor Indexer;
extern digital_out Pressure;
extern rotation Side;
extern controller Controller2;
extern rotation Flywheel;
extern digital_out Angler;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );