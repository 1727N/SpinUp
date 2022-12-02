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
// Roller               motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----
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

  std::cout << "IMU Calibrated" << std::endl << std::endl;
}

void driveForDist(double xDist, int timeOutLength, double maxSpeed){
  directDrive(xDist, timeOutLength, maxSpeed);
  waitUntil(runChassisControl == false);
}

void turnToAngle(int targetAngle, int timeOutLength){
  turnTo(targetAngle, timeOutLength);
  waitUntil(runChassisControl == false);
}

// bool flyWheelOn = false;

// void flywheelControl(int setVolt){
//   if (flyWheelOn){
//     FlyFront.spin(fwd, setVolt, volt);
//     FlyBack.spin(fwd, setVolt, volt);
//   }
//   else {
//     FlyFront.setStopping(coast);
//     FlyBack.setStopping(coast);
//     FlyFront.stop();
//     FlyBack.stop();
//   }
// }

void shoot(){
  Puncher.set(true);
  wait(300, msec);
  Puncher.set(false);
  wait(400, msec);
}

/* -------------------------- AUTON PROGRAMS -------------------------- */

//SKILLS
void autonSkills() {
  THETA_START = 0;
  //FW_MAX_POWER = 80;
  //FwVelocitySet( 93, 0.85 ); 

  directDrive(0.5, 300, 1);
  waitUntil(runChassisControl == false);

  Roller.spin(reverse);
  wait(700, msec);
  Roller.stop();
  
  directDrive(-10, 2000, 1);
  waitUntil(runChassisControl == false); 
  
  turnTo(270, 2000);
  waitUntil(runChassisControl == false);

  directDrive(12, 2000, 1);
  waitUntil(runChassisControl == false);

  FW_MAX_POWER = 70;
  FwVelocitySet( 70, 1 );

  Roller.spin(fwd);
  wait(700, msec);
  Roller.stop();
  
  directDrive(-30, 4000, 1);
  waitUntil(runChassisControl == false);

  turnToAngle(261, 2000);

  wait(200, msec);

  shoot();
  shoot();

  FwVelocitySet( 0, 0.00 );


  //waitUntil(runChassisControl == false);
}

void rollerStart(){
  THETA_START = 0;

  FW_MAX_POWER = 80;
  FwVelocitySet( 70, 1 );

  driveForDist(0.5, 500, 1);

  Roller.spin(fwd, 80, pct);
  wait(500, msec);
  Roller.stop();

  // setVoltage = 12;
  // flywheelSpin = true;
  

  directDrive(-2, 1000, 0.8);
  waitUntil(runChassisControl == false);

  turnTo(10, 2000);
  waitUntil(runChassisControl == false);

  wait(200, msec);

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

  FW_MAX_POWER = 80;
  FwVelocitySet( 70, 1 );

  driveForDist(0.5, 500, 1);

  Roller.spin(fwd, 80, pct);
  wait(500, msec);
  Roller.stop();

  // setVoltage = 12;
  // flywheelSpin = true;
  

  directDrive(-2, 1000, 0.8);
  waitUntil(runChassisControl == false);

  turnTo(10, 2000);
  waitUntil(runChassisControl == false);

  wait(200, msec);

  shoot();
  shoot();

  //SHOOT 2
  //
  FwVelocitySet( 0, 0.00 );

  turnTo(133, 2000);
  waitUntil(runChassisControl == false);
  driveForDist(65, 4000, 1);

  turnTo(90, 1000);
  driveForDist(3, 1, 1);

  Roller.spin(fwd, 80, pct);
  wait(500, msec);
  Roller.stop();
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
  DRIVER_CONTROL = false;

  //start odom
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);
  task flywheelTask(FwControlTask);


  waitUntil(!Inertial.isCalibrating());

  //flywheelAutonTest();
  autonSkills();
  //soloAWP();
  //rollerStart();
  //turnTo(8, 2000);
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
    Intake.spin(reverse, 100, pct);
  }
  else {
    Intake.setStopping(coast);
  }
}

void puncherControl(){
  if (flyWheelOn && FlyFront.velocity(rpm) > 400)
  {
    if (Controller1.ButtonUp.PRESSED){  
      Puncher.set(true);
      wait(200, msec);
      Puncher.set(false);
    }
  }
  else {
    if (Controller1.ButtonUp.PRESSED){  
      flyWheelOn = true;
    } 
  }
}

void catapultControl(){
  if (Controller1.ButtonY.pressing() && Controller1.ButtonLeft.pressing()){  
    Catapult.set(true);
  }
}

void rollerControl(){
  if (Controller1.ButtonA.pressing()){
    Roller.spin(fwd, 40, pct);
  }
  else {
    Roller.stop(coast);
  }
}

void usercontrol(void) {
  //flywheelTask.stop();
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
  task flywheelControlTask(FwControlTask);

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

    intakeControl();
    puncherControl();
    catapultControl();
    rollerControl();
    //flywheelControl(10);
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
