using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor FL;
extern motor FR;
extern motor BL;
extern motor BR;
extern inertial Inertial;
extern encoder Left;
extern encoder Side;
extern motor FlyFront;
extern motor FlyBack;
extern signature Vision__SIG_1;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern motor Intake;
extern digital_out Puncher;
extern digital_out Catapult;
extern encoder Right;
extern motor Indexer;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );