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
// Intake               motor         12              
// Puncher              digital_out   A               
// Catapult             digital_out   B               
// Right                encoder       C, D            
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "chassis-control.h"
#include "draw-field.h"
#include "flywheel.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

task odometryTask;
task drawFieldTask;
task chassisControlTask;
task intakeTask;
task flywheelTask;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Inertial.calibrate();

  while(Inertial.isCalibrating()) {
    task::sleep(100);
  }

  //roller: 0, nonroller: 270, AWP = 0
  Inertial.setHeading(0, degrees);

  std::cout << "IMU Calibrated" << std::endl << std::endl;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


bool flyWheelOn = false;

void flywheelControl(int setVolt){
  if (flyWheelOn){
    FlyFront.spin(fwd, setVolt, volt);
    FlyBack.spin(fwd, setVolt, volt);
  }
  else {
    FlyFront.setStopping(coast);
    FlyBack.setStopping(coast);
    FlyFront.stop();
    FlyBack.stop();
  }
}

void shoot(){
  Puncher.set(true);
  wait(500, msec);
  Puncher.set(false);
  wait(500, msec);
}

/* AUTON PROGRAMS */

//SKILLS
void autonSkills() {
  
}

motor_group Flywheel{FlyFront, FlyBack};

void flywheelAutonTest(){
  //flywheelControl(10);
  //flyWheelOn;

  Flywheel.setVelocity(500, rpm);
  Flywheel.spin(fwd);
  FlyFront.spin(fwd);
  FlyBack.spin(fwd);
}

void rollerStart(){
  THETA_START = 0;

  Intake.spin(fwd);
  wait(500, msec);
  Intake.stop();

  // setVoltage = 12;
  // flywheelSpin = true;
  FwVelocitySet( 100, 1 );

  directDrive(-2, 2000, 0.6);
  waitUntil(runChassisControl == false);

  turnTo(8, 1000);
  waitUntil(runChassisControl == false);

  wait(300, msec);

  shoot();
  shoot();

  //SHOOT 2
  //
  FwVelocitySet( 0, 0.00 );
  

  turnTo(135, 2000);
  waitUntil(runChassisControl == false);

  Intake.spin(fwd);
  directDrive(40, 5000, 1);
  waitUntil(runChassisControl == false);
  Intake.stop();

  turnTo(45, 1500);

  //SHOOT 3
  //
  shoot();
  shoot();
  shoot();

  
}

void nonRollerStart(){
  THETA_START = M_PI_2;

  Intake.spin(fwd);
  // go forward && intake 1
  directDrive(5, 2000, 1);
  waitUntil(runChassisControl == false);

  FwVelocitySet( 100, 0.80 );
  turnTo(225, 2000);
  waitUntil(runChassisControl == false);

  // shoot 3
  wait(2, sec);
  FwVelocitySet( 0, 0.00 );

  turnTo(135, 2000);
  waitUntil(runChassisControl == false);
  
  // go forward and intake 2
  directDrive(5, 2000, 1);
  waitUntil(runChassisControl == false);
  FwVelocitySet( 100, 0.80 );
  turnTo(225, 2000);
  waitUntil(runChassisControl == false);
  Intake.stop();
  
  //shoot 2
  wait(2, sec);
  FwVelocitySet( 0, 0.00 );

  //go back
  turnTo(315, 2000);
  waitUntil(runChassisControl == false);
  directDrive(10, 3000, 1);
  waitUntil(runChassisControl == false);
  turnTo(270, 1000);
  waitUntil(runChassisControl == false);

  Intake.spin(fwd);
  wait(500, msec);
  Intake.stop();
}

void soloAWP(){
  THETA_START = 0;
  directDrive(1, 500, 0.5);
  waitUntil(runChassisControl == false);

  Intake.spin(fwd);
  wait(500, msec);
  Intake.stop();

  // setVoltage = 12;
  // flywheelSpin = true;
  FwVelocitySet( 100, 0.93 );

  directDrive(-2, 2000, 0.6);
  waitUntil(runChassisControl == false);

  turnTo(8, 1000);
  waitUntil(runChassisControl == false);

  wait(500, msec);

  shoot();
  shoot();

  //SHOOT 2
  //
  FwVelocitySet( 0, 0.00 );
  

  turnTo(130, 2000);
  waitUntil(runChassisControl == false);

  Intake.spin(fwd);
  directDrive(130, 8000, 1);
  waitUntil(runChassisControl == false);
  Intake.stop();

  turnTo(90, 1000);
  waitUntil(runChassisControl == false);

  directDrive(10, 1000, 0.5);
  waitUntil(runChassisControl == false);

  Intake.spin(fwd);
  wait(500, msec);
  Intake.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  //reset sensors
  Left.resetRotation();
  Right.resetRotation();
  Side.resetRotation();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();

  //start odom
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);
  task flywheelControlTask(FwControlTask);

  wait(800, msec);

  //flywheelAutonTest();

  //autonSkills();
  //rollerStart();
  //nonRollerStart();
  soloAWP();
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
  if (Controller1.ButtonR1.pressing()){
   intakeTrue = true;
  }
  else if (Controller1.ButtonR2.pressing()){
   intakeTrue = false;
   Intake.stop();
  }
  if (intakeTrue){
   Intake.spin(fwd, intakePct, pct);
  }
  if(Controller1.ButtonB.PRESSED) {
    outTakeTrue = !outTakeTrue;
    Intake.stop();
  }
  if (outTakeTrue){
    Intake.spin(reverse, 60, pct);
  }
  else {
    Intake.setStopping(coast);
  }
  if (Controller1.ButtonX.pressing()){
    Intake.spin(reverse, 40, pct);
  }
}

void puncherControl(){
  if (Controller1.ButtonUp.PRESSED){  
    Puncher.set(true);
    wait(700, msec);
    Puncher.set(false);
  }
}

void catapultControl(){
  if (Controller1.ButtonY.PRESSED){  
    Catapult.set(true);
  }
}

double flyError = 0;
double flyPrevError = 0;
double flyMaxError = 10;

double flykP = 0.5;
double flykI = 0;
double flykD = 0;

double flyPowerPID = 0;

void flyPID(int targetRPM) {
  //Error is equal to the difference between the current facing direction and the target direction
  flyError = targetRPM - FlyFront.velocity(rpm);

  //use integral if close enough to target
  // if(fabs(flyError) < flyIntegralBound) {
  //   flyIntegral += flyError;
  // }
  // else {
  //   flyIntegral = 0;
  // }

  //reset integral if we pass the target
  // if(flyError * flyPrevError < 0) {
  //   turnIntegral = 0;
  // } 

  flyPrevError = flyError - flyPrevError;

  flyPrevError = flyError;

  flyPowerPID = flyPowerPID + (flyError * flykP  + flyPrevError * flykD);

  //Limit power output to 12V
    if(flyPowerPID > 12) {
      flyPowerPID = 12;
    }
  

  if(fabs(flyError) < flyMaxError) {
    flyPowerPID = 0;
  }

  FlyFront.spin(fwd, flyPowerPID, volt);
  FlyBack.spin(fwd, flyPowerPID, volt);
}

void usercontrol(void) {
  Left.resetRotation();
  Right.resetRotation();
  Side.resetRotation();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();

  double driveAmt;
  double turnAmt;
  double strafeAmt;

  FL.setBrake(brakeType::brake);
  FR.setBrake(brakeType::brake);
  BL.setBrake(brakeType::brake);
  BR.setBrake(brakeType::brake);
  
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);
  //task flywheelControlTask(FwControlTask);

  FlyFront.setBrake(coast);
  FlyBack.setBrake(coast);

  while (1) {
    float maxSpeed = 100;
    float leftPct = (Controller1.Axis3.position())/maxSpeed;
    float rightPct = (Controller1.Axis2.position())/maxSpeed;

    float leftNewPct = leftPct * leftPct * leftPct * 100;
    float rightNewPct = rightPct * rightPct * rightPct * 100;

    FL.spin(reverse, rightNewPct, pct);
    BL.spin(reverse, rightNewPct, pct);
    FR.spin(reverse, leftNewPct, pct);
    BR.spin(reverse, leftNewPct, pct);

    /* DRIVE */
    // Brain.Screen.printAt( 10, 125, "Left %6.1f", Left.position(deg));
    // Brain.Screen.printAt( 10, 200, "Back %6.1f", Side.position(deg));

    // driveAmt = exponentialDrive(Controller1.Axis3.value());
    // turnAmt = exponentialDrive(Controller1.Axis1.value());
    // strafeAmt = exponentialDrive(Controller1.Axis4.value());

    // FL.spin(directionType::fwd, driveAmt + turnAmt + strafeAmt, velocityUnits::pct);
    // FR.spin(directionType::fwd, driveAmt - turnAmt - strafeAmt, velocityUnits::pct);
    // BL.spin(directionType::fwd, driveAmt + turnAmt - strafeAmt, velocityUnits::pct);
    // BR.spin(directionType::fwd, driveAmt - turnAmt + strafeAmt, velocityUnits::pct);

    intakeControl();
    puncherControl();
    catapultControl();
    flywheelControl(10);
    //flyPID(400);

    if (Controller1.ButtonL1.PRESSED){
      flyWheelOn = true;
      //FwVelocitySet( 100, 0.85 );
      //FlyBack.spin(fwd);
    }
    if (Controller1.ButtonL2.PRESSED){
      flyWheelOn = false;
      //FwVelocitySet( 0, 0 );      
    }

    //std::cout << FlyFront.velocity(rpm) << std::endl << std::endl;

    // if(abs(Controller1.Axis3.value()) < 5 && abs(Controller1.Axis1.value()) < 5 && abs(Controller1.Axis4.value()) < 5) {
    //   FL.stop(brakeType::brake);
    //   FR.stop(brakeType::brake);
    //   BL.stop(brakeType::brake);
    //   BR.stop(brakeType::brake);
    // }

    // if (Controller1.ButtonX.PRESSED){
    //   turnTo(currentAbsoluteOrientation + M_PI_2, 5000);
    // }

    // std::cout << Brain.Timer.value() << "," 
    //     << FlyFront.velocity(rpm) << "," <<  FlyFront.torque(Nm) << "," 
    //     << FlyFront.current() << "," << FlyFront.voltage(volt)
    //     << std::endl; 

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

  /*
  FL.setBrake(brakeType::brake);
  FR.setBrake(brakeType::brake);
  BL.setBrake(brakeType::brake);
  BR.setBrake(brakeType::brake);

  while (1) {
    float maxSpeed = 100;
    float leftPct = (Controller1.Axis3.position())/maxSpeed;
    float rightPct = (Controller1.Axis2.position())/maxSpeed;

    float leftNewPct = leftPct * leftPct *leftPct*100;
    float rightNewPct = rightPct *rightPct *rightPct*100;

    FR.spin(fwd, rightNewPct, pct);
    BR.spin(fwd, rightNewPct, pct);
    FL.spin(fwd, leftNewPct, pct);
    BL.spin(fwd, leftNewPct, pct);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  */
}

//
// Main will set up the competition functions and callbacks.
//
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
    //flywheelControl(10);
    // Brain.Screen.printAt( 10, 125, "Left %6.1f", Left.position(deg));
    // Brain.Screen.printAt( 10, 200, "Back %6.1f", Side.position(deg));

    // Brain.Screen.printAt( 10, 75, "Angle %6.1f", Inertial.heading());
    wait(100, msec);
  }
}
