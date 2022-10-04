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
// FrontLeftDrive       motor         11              
// FrontRightDrive      motor         1               
// BackLeftDrive        motor         19              
// BackRightDrive       motor         9               
// STrack               rotation      20              
// Inertial6            inertial      5               
// Left                 encoder       G, H            
// Right                encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "chassis-control.h"
#include "draw-field.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

task odometryTask;
task drawFieldTask;
task chassisControlTask;
task intakeTask;
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

  Inertial6.calibrate();

  while(Inertial6.isCalibrating()) {
    task::sleep(100);
  }

  Inertial6.setHeading(270, rotationUnits::deg);

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





/* AUTON PROGRAMS */

//SKILLS
void autonSkills() {
  // driveTo(56.5, 18.5, 90, 1500, 0.6);
  // waitUntil(runChassisControl == false);
  turnToPoint(0, 0, 3000, 0.8);
  wait(3000, msec);
  //waitUntil(runChassisControl == false);
}




void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  //reset rotation sensors
  Left.resetRotation();
  Right.resetRotation();
  STrack.resetPosition();

  FrontLeftDrive.resetRotation();
  FrontRightDrive.resetRotation();
  BackLeftDrive.resetRotation();
  BackRightDrive.resetRotation();

  //start the odometry
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);

  wait(1500, msec);

  autonSkills();
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

double leftError = 0;
double rightError = 0;
double intakekPDriver = 1.0;

void usercontrol(void) {
  Left.resetRotation();
  Right.resetRotation();
  STrack.resetPosition();

  FrontLeftDrive.resetRotation();
  FrontRightDrive.resetRotation();
  BackLeftDrive.resetRotation();
  BackRightDrive.resetRotation();

  double driveAmt;
  double turnAmt;
  double strafeAmt;

  FrontLeftDrive.setBrake(brakeType::brake);
  FrontRightDrive.setBrake(brakeType::brake);
  BackLeftDrive.setBrake(brakeType::brake);
  BackRightDrive.setBrake(brakeType::brake);

  Inertial6.calibrate();

  while(Inertial6.isCalibrating()) {
    task::sleep(100);
  }

  std::cout << "IMU Calibrated" << std::endl << std::endl;

  Inertial6.setHeading(270, rotationUnits::deg);
  
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);

  while (1) {
    /* DRIVE */
    //Brain.Screen.printAt( 10, 125, "Left %6.1f", Left.position(deg));
    //Brain.Screen.printAt( 10, 200, "Right %6.1f", Right.position(deg));

    driveAmt = 0.4 * exponentialDrive(Controller1.Axis3.value());
    turnAmt = 0.4 * exponentialDrive(Controller1.Axis1.value());
    strafeAmt = exponentialDrive(Controller1.Axis4.value());

    FrontLeftDrive.spin(directionType::fwd, driveAmt + turnAmt + strafeAmt, velocityUnits::pct);
    FrontRightDrive.spin(directionType::fwd, driveAmt - turnAmt - strafeAmt, velocityUnits::pct);
    BackLeftDrive.spin(directionType::fwd, driveAmt + turnAmt - strafeAmt, velocityUnits::pct);
    BackRightDrive.spin(directionType::fwd, driveAmt - turnAmt + strafeAmt, velocityUnits::pct);

    if(abs(Controller1.Axis3.value()) < 5 && abs(Controller1.Axis1.value()) < 5 && abs(Controller1.Axis4.value()) < 5) {
      FrontLeftDrive.stop(brakeType::brake);
      FrontRightDrive.stop(brakeType::brake);
      BackLeftDrive.stop(brakeType::brake);
      BackRightDrive.stop(brakeType::brake);
    }

    if (Controller1.ButtonX.PRESSED){
      turnToPoint(0, 0, 2000, 0.8);
      waitUntil(runChassisControl == false);
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

  /*
  FrontLeftDrive.setBrake(brakeType::brake);
  FrontRightDrive.setBrake(brakeType::brake);
  BackLeftDrive.setBrake(brakeType::brake);
  BackRightDrive.setBrake(brakeType::brake);

  while (1) {
    float maxSpeed = 100;
    float leftPct = (Controller1.Axis3.position())/maxSpeed;
    float rightPct = (Controller1.Axis2.position())/maxSpeed;

    float leftNewPct = leftPct * leftPct *leftPct*100;
    float rightNewPct = rightPct *rightPct *rightPct*100;

    FrontRightDrive.spin(fwd, rightNewPct, pct);
    BackRightDrive.spin(fwd, rightNewPct, pct);
    FrontLeftDrive.spin(fwd, leftNewPct, pct);
    BackLeftDrive.spin(fwd, leftNewPct, pct);

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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
