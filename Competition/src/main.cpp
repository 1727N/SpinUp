/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       LUX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Program                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      17              
// Left                 encoder       G, H            
// Side                 encoder       E, F            
// FlyFront             motor         5               
// FlyBack              motor         6               
// Vision               vision        3               
// Intake               motor         16              
// IndexerPiston              digital_out   C               
// Endgame              digital_out   B               
// Indexer              motor         15              
// Pressure             digital_out   A               
// IntakePump           digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "chassis-control.h"
#include "draw-field.h"
#include "flywheel.h"
#include <iostream>

using namespace vex;

competition Competition;

task odometryTask;
task drawFieldTask;
task chassisControlTask;
task flywheelTask;

void pre_auton(void) {
  vexcodeInit();

  Inertial.calibrate();

  while(Inertial.isCalibrating()) {
    task::sleep(100);
  }

  //roller: 180, nonroller: 270, AWP = 180
  Inertial.setHeading(180, degrees);

  wait(800, msec);
}

/*---------------------------------------------------------- HELPER METHODS ----------------------------------------------------------*/

void driveForDist(double xDist, int timeOutLength, double maxSpeed){
  directDrive(xDist, timeOutLength, maxSpeed);
  waitUntil(runChassisControl == false);
}

void turnToAngle(int targetAngle, int timeOutLength){
  turnTo(targetAngle, timeOutLength);
  waitUntil(runChassisControl == false);
}

void moveToPoint(double xPos, double yPos, int timeOutLength, double maxSpeed){
  turnToPoint(xPos, yPos, 1500, 1);
  waitUntil(runChassisControl == false);
  driveTo(xPos, yPos, 1, timeOutLength, maxSpeed);
  waitUntil(runChassisControl == false);
}

void rollFor(int timeRoll){
  Intake.spin(reverse, 60, pct);
  wait(timeRoll, msec);
  Intake.stop();
}

void shoot(){
  Indexer.setVelocity(100, pct);
  Intake.setVelocity(90, pct);
  Indexer.spin(reverse);
  Intake.spin(fwd);
  IndexPiston.set(true);
  Pressure.set(true);
  wait(300, msec);
  Pressure.set(false);
  IndexPiston.set(false);
  Indexer.stop();
  Intake.stop();
  wait(500, msec);
}

/*---------------------------------------------------------- AUTONOMOUS PROGRAMS ----------------------------------------------------------*/

// SKILLS
void autonSkills() {
  FL.spinFor(reverse, 180, degrees, false);
  FR.spinFor(reverse, 180, degrees, false);
  BL.spinFor(reverse, 180, degrees, false);
  BR.spinFor(reverse, 180, degrees, true);

  Indexer.spin(reverse, 60, pct);
  wait(800, msec);
  Indexer.stop();

  FL.spinFor(fwd, 560, degrees, false);
  FR.spinFor(fwd, 560, degrees, false);
  BL.spinFor(fwd, 560, degrees, false);
  BR.spinFor(fwd, 560, degrees, true);

  turnToAngle(270, 4000);

  FL.spinFor(reverse, 650, degrees, false);
  FR.spinFor(reverse, 650, degrees, false);
  BL.spinFor(reverse, 650, degrees, false);
  BR.spinFor(reverse, 650, degrees, true);

  Indexer.spin(reverse, 60, pct);
  wait(800, msec);
  Indexer.stop();

  FL.spinFor(fwd, 360, degrees, false);
  FR.spinFor(fwd, 360, degrees, false);
  BL.spinFor(fwd, 360, degrees, false);
  BR.spinFor(fwd, 360, degrees, true);

  turnToAngle(315, 4000);
}

void rollerStart(){
  THETA_START = 0;

  driveForDist(-4.5, 800, 1);

  Indexer.spin(reverse, 100, pct);
  wait(400, msec);
  Indexer.stop();

  driveForDist(8, 1300, 1);

  turnToAngle(317, 1000);

  FlyFront.spin(reverse, 450, rpm);
  FlyBack.spin(reverse, 450, rpm);

  driveForDist(63, 2000, 1);

  turnToAngle(38, 1000);
  shoot();

  Intake.spin(fwd, 100, pct);
  Indexer.spin(fwd, 100, pct);
  wait(800, msec);

  Intake.stop();
  Indexer.stop();

  shoot();

  turnToAngle(0, 800);
  driveForDist(-45, 2000, 1);
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

  driveForDist(-4.5, 800, 1);

  Indexer.spin(reverse, 100, pct);
  wait(400, msec);
  Indexer.stop();

  driveForDist(8, 1300, 1);

  turnToAngle(135, 1300);

  int speed = 2350;

  FwVelocitySet(speed, 0.85);

  // FlyFront.spin(reverse, 460, rpm);
  // FlyBack.spin(reverse, 460, rpm);

  driveForDist(61, 2000, 1);

  turnToAngle(220, 1300);
  waitUntil(-Flywheel.velocity(rpm) > speed-15  && -Flywheel.velocity(rpm) < speed+15);
  shoot();
  waitUntil(-Flywheel.velocity(rpm) > speed-15  && -Flywheel.velocity(rpm) < speed+15);
  shoot();

  wait(500, msec);

  FwVelocitySet(0, 0);

  turnToAngle(313, 1300);

  driveForDist(-59, 1700, 1);

  turnToAngle(270, 1300);

  driveForDist(-15, 1300, 1);

  Indexer.spin(reverse, 100, pct);
  wait(700, msec);
  Indexer.stop();
}

void tuning(){
  //FwVelocitySet(2500, 0.85);
  // Intake.spin(fwd, 100, pct);
  // Indexer.spin(fwd, 100, pct);
  // driveForDist(46, 3000, 0.6);
  // turnToAngle(270, 1300);

  // waitUntil(-Flywheel.velocity(rpm) > 2490  && -Flywheel.velocity(rpm) < 2510);
  // shoot();
  // waitUntil(-Flywheel.velocity(rpm) > 2490  && -Flywheel.velocity(rpm) < 2510);
  // shoot();

  // driveForDist(-39, 3000, 1);
  // turnToAngle(0, 3000);
  // driveForDist(-39, 3000, 1);

  // moveToPoint(60, 70, 3000, 0.5);
}

/*---------------------------------------------------------- AUTONOMOUS CONTROL ----------------------------------------------------------*/

void autonomous(void) {
  Left.resetRotation();
  Side.resetPosition();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();
  DRIVER_CONTROL = false;

  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);
  task flywheelTask(FwControlTask);

  waitUntil(!Inertial.isCalibrating());

  //autonSkills();
  //tuning();
  soloAWP();
  //rollerStart();
  //nonRollerStart();
}

/*---------------------------------------------------------- DRIVER METHODS ----------------------------------------------------------*/

double exponentialDrive(double controllerValue) {
  return pow(controllerValue, 3) / pow(100, 2);
}

// INTAKE

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
   outTakeTrue = false;
   Intake.stop();
   Indexer.stop();
  }
  if (intakeTrue){
   Intake.spin(fwd, intakePct, pct);
   Indexer.spin(fwd, intakePct, pct);
  }
  if(Controller1.ButtonB.PRESSED) {
    outTakeTrue = !outTakeTrue;
    Intake.stop();
    Indexer.stop();
  }
  if (outTakeTrue){
    Intake.spin(reverse, 100, pct);
    Indexer.spin(reverse, 100, pct);
    Controller1.rumble(".");
  }
  else {
    Intake.setStopping(coast);
    Indexer.setStopping(coast);
  }
}

// SHOOTING

void flywheelControl(){
  if (flyWheelOn){
      // FL.setBrake(brake);
      // FR.setBrake(brake);
      // BR.setBrake(brake);
      // BL.setBrake(brake);
    }
    else {
      FL.setBrake(coast);
      FR.setBrake(coast);
      BR.setBrake(coast);
      BL.setBrake(coast);
    }

    if (Controller1.ButtonL1.PRESSED){
      flyWheelOn = true;
      FwVelocitySet( 2000, 0.67 );  
    }
    if (Controller1.ButtonL2.PRESSED){
      flyWheelOn = false;
      FwVelocitySet( 0, 0 );      
    }
}

void shooterControl(){
  if (Controller1.ButtonUp.PRESSED){
    Indexer.setVelocity(100, pct);
    Intake.setVelocity(90, pct);
    
    if (flyWheelOn && -FlyFront.velocity(rpm) > 270)
    {
      Indexer.spin(reverse);
      Intake.spin(fwd);
      flywheelVoltage = 10;
      IndexPiston.set(true);
      Pressure.set(true);
      wait(1500, msec);
      Pressure.set(false);
      IndexPiston.set(false);
      Indexer.stop();
      Intake.stop();
      flywheelVoltage = 7;
    }
    else {  
      flyWheelOn = true; 
    }
  }
}

// ENDGAME

void endgameControl(){
  if (Controller1.ButtonY.pressing() && Controller1.ButtonRight.pressing()){  
    Endgame.set(true);
  }
}
    
/*---------------------------------------------------------- USER CONTROL ----------------------------------------------------------*/

void tuneTurns(){
  if (Controller1.ButtonX.PRESSED){
    turnkP += 1;
  }
  if (Controller1.ButtonA.PRESSED){
    turnkP -= 1;
  }
  if (Controller1.ButtonY.PRESSED){
    turnkD += 1;
  }
  if (Controller1.ButtonB.PRESSED){
    turnkD -= 1;
  }  
  
  std::cout << turnkP << std::endl << std::endl;
  std::cout << turnkI << std::endl << std::endl;
  std::cout << turnkD << std::endl << std::endl;
}

void tuneDrive(){
  if (Controller1.ButtonX.PRESSED){
    drivekP += 1;
  }
  if (Controller1.ButtonA.PRESSED){
    drivekP -= 1;
  }
  if (Controller1.ButtonY.PRESSED){
    drivekD += 1;
  }
  if (Controller1.ButtonB.PRESSED){
    drivekD -= 1;
  }  
  
  std::cout << drivekP << std::endl << std::endl;
  std::cout << drivekI << std::endl << std::endl;
  std::cout << drivekD << std::endl << std::endl;
}

void tuneFlywheel(){
  if (Controller1.ButtonX.PRESSED){
    flykP += 1;
  }
  if (Controller1.ButtonA.PRESSED){
    flykP -= 1;
  }
  if (Controller1.ButtonY.PRESSED){
    flykD += 1;
  }
  if (Controller1.ButtonB.PRESSED){
    flykD -= 1;
  }
}

void tuneTBH(){
  if (Controller1.ButtonX.PRESSED){
    gain += 0.00001;
  }
  if (Controller1.ButtonA.PRESSED){
    gain -= 0.00001;
  }

  if (Brain.Timer.value() < 10){
    std::cout << Brain.Timer.value() << "," << -Flywheel.velocity(rpm) << std::endl; 
  }

  //std::cout << gain << std::endl << std::endl;
}

void tankDrive(){
  float maxSpeed = 100;

  int exp = 3;

  float leftPct = (Controller1.Axis3.position());
  float rightPct = (Controller1.Axis2.position());

  float leftNewPct = pow(leftPct, exp)/pow(maxSpeed, exp-1);
  float rightNewPct = pow(rightPct, exp)/pow(maxSpeed, exp-1);

  FL.spin(fwd, leftNewPct, pct);
  BL.spin(fwd, leftNewPct, pct);
  FR.spin(fwd, rightNewPct, pct);
  BR.spin(fwd, rightNewPct, pct);
}

void arcadeDrive(){
  float arcadeLeftPct = (Controller1.Axis3.position() + Controller1.Axis1.position() * 0.6);
  float arcadeRightPct = (Controller1.Axis3.position() - Controller1.Axis1.position()* 0.6);

  FL.spin(fwd, arcadeLeftPct * 0.5, pct);
  BL.spin(fwd, arcadeLeftPct * 0.5, pct);
  FR.spin(fwd, arcadeRightPct * 0.5, pct);
  BR.spin(fwd, arcadeRightPct * 0.5, pct);
}

void usercontrol(void) {
  DRIVER_CONTROL = true;
  Endgame.set(false);

  Left.resetRotation();
  Side.resetPosition();

  FL.setBrake(brakeType::coast);
  FR.setBrake(brakeType::coast);
  BL.setBrake(brakeType::coast);
  BR.setBrake(brakeType::coast);
  
  // task odometryTask(positionTracking);
  // task drawFieldTask(drawField);
  // task chassisControlTask(chassisControl);
  task flywheelTask(FwControlTask);

  FlyFront.setBrake(coast);
  FlyBack.setBrake(coast);

  Brain.Screen.clearScreen();

  while (1) {
    tankDrive();
    //arcadeDrive();

    //tuneDrive();

    // if (Controller1.ButtonLeft.PRESSED){
    //   turnToPoint(30, 30, 1300, 1);
    //   waitUntil(runChassisControl == false);
    //   moveToPoint(70, 70, 3000, 0.7);
    // }
    // if (Controller1.ButtonLeft.PRESSED){
    //   turnToAngle(0, 1300);
    // }
    // if (Controller1.ButtonRight.PRESSED){
    //   turnToAngle(180, 1300);
    // }
    // if (Controller1.ButtonUp.PRESSED)
    // {
    //   driveForDist(-39, 2000, 1);
    // }
    // if (Controller1.ButtonDown.PRESSED){
    //   driveForDist(-24, 3000, 1);
    //   //turnToAngle(0, 5000);
    // }

    intakeControl();
    shooterControl();
    flywheelControl();
    endgameControl();

    wait(20, msec);
  }
}

int main() {
  Competition.bStopAllTasksBetweenModes = true;
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // std::cout << "----------------------------------------------------------------------------------------------------------------" << std::endl << std::endl;

  while(Inertial.isCalibrating()) {
    task::sleep(100);
  }

  Controller1.rumble("-");
  
  while (true) {
    wait(100, msec);
  }
}
