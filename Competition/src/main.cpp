/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         11              
// FR                   motor         1               
// BL                   motor         19              
// BR                   motor         9               
// Inertial             inertial      17              
// Left                 encoder       G, H            
// Side                 encoder       E, F            
// FlyFront             motor         5               
// FlyBack              motor         6               
// Vision               vision        3               
// Intake               motor         16              
// Puncher              digital_out   A               
// Catapult             digital_out   B               
// Right                encoder       C, D            
// Indexer              motor         15              
// ---- END VEXCODE CONFIGURED DEVICES ----
//hello guys
#include "chassis-control.h"
#include "draw-field.h"
#include "flywheel.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

motor_group Flywheel{FlyFront, FlyBack};

task odometryTask;
task drawFieldTask;
task chassisControlTask;
task intakeTask;
task flywheelTask;

void pre_auton(void) {
  vexcodeInit();

  Inertial.calibrate();

  while(Inertial.isCalibrating()) {
    task::sleep(100);
  }

  //roller: 0, nonroller: 270, AWP = 0
  Inertial.setHeading(0, degrees);
}

void driveForDist(double xDist, int timeOutLength, double maxSpeed){
  directDrive(xDist, timeOutLength, maxSpeed);
  waitUntil(runChassisControl == false);
}

void turnToAngle(int targetAngle, int timeOutLength){
  turnTo(targetAngle, timeOutLength);
  waitUntil(runChassisControl == false);
}

void shoot(){
  Puncher.set(true);
  wait(1300, msec);
  Puncher.set(false);
  wait(400, msec);
}

void rollFor(int timeRoll){
  Intake.spin(reverse, 60, pct);
  wait(timeRoll, msec);
  Intake.stop();
}

/* -------------------------- AUTON PROGRAMS -------------------------- */

//SKILLS
void autonSkills() {

}

void rollerStart(){
  THETA_START = 0;

  FW_MAX_POWER = 80;
  //FwVelocitySet( 70, 1 );

  driveForDist(0.5, 500, 1);

  Intake.spin(fwd, 60, pct);
  wait(300, msec);
  Intake.stop();

  directDrive(-2, 1000, 0.8);
  waitUntil(runChassisControl == false);

  turnTo(10, 2000);
  waitUntil(runChassisControl == false);

  wait(200, msec);

  //shoot();
  //shoot(); 

  //FwVelocitySet( 0, 0.00 ); 

  // Intake.spin(fwd);

  // turnTo(130, 2000);
  // waitUntil(runChassisControl == false);
  // FwVelocitySet( 0, 0.00 ); 

  // Intake.spin(fwd);
  // directDrive(40, 5000, 1);
  // waitUntil(runChassisControl == false);
  // Intake.stop();

  // turnTo(45, 1500);

  // shoot();
  // shoot();
  // shoot();
}

void nonRollerStart(){
  THETA_START = 0;

  driveForDist(13, 3000, 1);
  turnToAngle(270, 2000);
  driveForDist(2, 3000, 1);

  Intake.spin(fwd);
  wait(500, msec);
  Intake.stop();
}

void soloAWP(){
  THETA_START = 0;

  FW_MAX_POWER = 80;
  FwVelocitySet( 70, 1 );

  driveForDist(0.5, 500, 1);

  Intake.spin(fwd, 80, pct);
  wait(500, msec);
  Intake.stop();

  directDrive(-2, 1000, 0.8);
  waitUntil(runChassisControl == false);

  turnTo(10, 2000);
  waitUntil(runChassisControl == false);

  wait(200, msec);

  shoot();
  shoot();

  FwVelocitySet( 0, 0.00 );

  turnTo(133, 2000);
  waitUntil(runChassisControl == false);
  driveForDist(65, 4000, 1);

  turnTo(90, 1000);
  driveForDist(3, 1, 1);

  Intake.spin(fwd, 80, pct);
  wait(500, msec);
  Intake.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  //reset sensors
  Left.resetRotation();
  //Right.resetRotation();
  Side.resetRotation();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();
  DRIVER_CONTROL = false;

  //start odom
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);
  task flywheelTask(FwControlTask);
  // flywheelTask = task(FwControlTask);


  waitUntil(!Inertial.isCalibrating());

  //autonSkills();
  //soloAWP();
  //rollerStart();
  //nonRollerStart();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double exponentialDrive(double controllerValue) {
  return pow(controllerValue, 3) / pow(100, 2);
}

bool intakeTrue = false;
bool outTakeTrue = false;
const int intakePct = 100;

void intakeControl(){
  Intake.setVelocity( intakePct, pct);
  Indexer.setVelocity( intakePct, pct);
  if (Controller1.ButtonR1.pressing()){
   intakeTrue = true;
  }
  else if (Controller1.ButtonR2.pressing()){
   intakeTrue = false;
   Intake.stop();
   Indexer.stop();
  }
  if (intakeTrue){
   Intake.spin(fwd, intakePct, pct);
   Indexer.spin(fwd, intakePct, pct);
  }
  if(Controller1.ButtonB.PRESSED) {
    //outTakeTrue = !outTakeTrue;
    Intake.stop();
    Indexer.stop();
  }
  if (outTakeTrue){
    Intake.spin(reverse, 100, pct);
    Indexer.spin(reverse, 100, pct);
  }
  else {
    Intake.setStopping(coast);
    Indexer.setStopping(coast);
  }
}

void puncherControl(){
  if (Controller1.ButtonUp.PRESSED){
    Indexer.setVelocity(100, pct);
    Intake.setVelocity(50, pct);
    Indexer.spin(reverse);
    Intake.spin(fwd);
    if (flyWheelOn && -FlyFront.velocity(rpm) > 300)
    {
      Puncher.set(true);
      wait(2000, msec);
      Puncher.set(false);
      Indexer.stop();
      Intake.stop();
    }
    else {  
      flyWheelOn = true; 
    }
  }
}

void catapultControl(){
  if (Controller1.ButtonY.pressing() && Controller1.ButtonRight.pressing()){  
    Catapult.set(true);
  }
}

void rollerControl(){
  if (Controller1.ButtonA.pressing()){
    Indexer.spin(fwd, 40, pct);
  }
}

void usercontrol(void) {
  //flywheelTask.stop();

  //https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/35

  DRIVER_CONTROL = true;

  Left.resetRotation();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();

  FL.setBrake(brakeType::brake);
  FR.setBrake(brakeType::brake);
  BL.setBrake(brakeType::brake);
  BR.setBrake(brakeType::brake);
  
  //task odometryTask(positionTracking);
  //task drawFieldTask(drawField);
  //task chassisControlTask(chassisControl);
  flywheelTask = task(FwControlTask);

  FlyFront.setBrake(coast);
  FlyBack.setBrake(coast);

  while (1) {
    float maxSpeed = 100;
    // float leftPct = (Controller1.Axis3.position())/maxSpeed;
    // float rightPct = (Controller1.Axis2.position())/maxSpeed;

    // float leftNewPct = leftPct * leftPct * leftPct * 100;
    // float rightNewPct = rightPct * rightPct * rightPct * 100;

    int exp = 2;

    float leftPct = (Controller1.Axis3.position());
    float rightPct = (Controller1.Axis2.position());

    float leftNewPct = pow(leftPct, exp)/pow(maxSpeed, exp-1);
    float rightNewPct = pow(rightPct, exp)/pow(maxSpeed, exp-1);


    FL.spin(fwd, leftNewPct, pct);
    BL.spin(fwd, leftNewPct, pct);
    FR.spin(fwd, rightNewPct, pct);
    BR.spin(fwd, rightNewPct, pct);

    intakeControl();
    puncherControl();
    catapultControl();
    rollerControl();

    if (Controller1.ButtonL1.PRESSED){
      flyWheelOn = true;
      FwVelocitySet( 100, 0.85 );
      //FlyBack.spin(fwd);
    }
    if (Controller1.ButtonL2.PRESSED){
      flyWheelOn = false;
      FwVelocitySet( 0, 0 );      
    }

    //std::cout << FlyFront.velocity(rpm) << std::endl << std::endl;

    // std::cout << Brain.Timer.value() << "," 
    //     << FlyFront.velocity(rpm) << "," <<  FlyFront.torque(Nm) << "," 
    //     << FlyFront.current() << "," << FlyFront.voltage(volt)
    //     << std::endl; 

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  std::cout << "----------------------------------------------------------------------------------------------------------------" << std::endl << std::endl;

  Inertial.calibrate();

  while(Inertial.isCalibrating()) {
    task::sleep(100);
  }

  std::cout << "IMU Calibrated" << std::endl << std::endl;
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    // Brain.Screen.printAt( 10, 125, "Left %6.1f", Left.position(deg));
    // Brain.Screen.printAt( 10, 200, "Back %6.1f", Side.position(deg));

    // Brain.Screen.printAt( 10, 75, "Angle %6.1f", Inertial.heading());
    wait(100, msec);
  }
}
