// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      18              
// Left                 encoder       G, H            
// Intake               motor         9               
// Endgame              digital_out   C               
// Catapult             motor         13              
// MR                   motor         19              
// ML                   motor         12              
// PistonBoostL         digital_out   A               
// PistonBoostR         digital_out   B               
// Side                 encoder       E, F            
// Limit                limit         D               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      18              
// Left                 encoder       G, H            
// Intake               motor         9               
// Endgame              digital_out   C               
// Catapult             motor         13              
// MR                   motor         19              
// ML                   motor         12              
// PistonBoostL         digital_out   A               
// PistonBoostR         digital_out   B               
// Side                 encoder       E, F            
// Limit                limit         D               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      18              
// Left                 encoder       G, H            
// Intake               motor         9               
// Endgame              digital_out   C               
// Catapult             motor         13              
// MR                   motor         19              
// ML                   motor         12              
// PistonBoostL         digital_out   A               
// PistonBoostR         digital_out   B               
// Side                 encoder       E, F            
// Limit                limit         D               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      18              
// Left                 encoder       G, H            
// Intake               motor         9               
// Endgame              digital_out   C               
// Catapult             motor         13              
// MR                   motor         19              
// ML                   motor         12              
// PistonBoostL         digital_out   A               
// PistonBoostR         digital_out   B               
// Side                 encoder       E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      18              
// Left                 encoder       G, H            
// Intake               motor         9               
// Endgame              digital_out   C               
// Catapult             motor         13              
// MR                   motor         19              
// ML                   motor         12              
// PistonBoostL         digital_out   A               
// PistonBoostR         digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// FR                   motor         10              
// BL                   motor         11              
// BR                   motor         20              
// Inertial             inertial      18              
// Left                 encoder       G, H            
// Intake               motor         9               
// Endgame              digital_out   C               
// Side                 rotation      2               
// Catapult             motor         13              
// MR                   motor         19              
// ML                   motor         12              
// PistonBoostL         digital_out   A               
// PistonBoostR         digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "chassis-control.h"
#include "draw-field.h"
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

int targetSpeed;

void shoot(){

}

int shotClock;

void shoot(int timeFor){

}


/*---------------------------------------------------------- AUTONOMOUS PROGRAMS ----------------------------------------------------------*/

// SKILLS

/*---------------------------------------------------------- AUTONOMOUS CONTROL ----------------------------------------------------------*/

void autonomous(void) {
  Left.resetRotation();
  Side.resetRotation();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();

  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  task chassisControlTask(chassisControl);

  waitUntil(!Inertial.isCalibrating());
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
  Intake.setVelocity( intakePct, pct);
  if (Controller1.ButtonY.PRESSED){
    Intake.spinFor(fwd, 400, msec);
  }
  if (Controller1.ButtonR1.pressing()){
   intakeTrue = true;
  }
  else if (Controller1.ButtonR2.pressing()){
   intakeTrue = false;
   outTakeTrue = false;
   Intake.stop();
   Intake.stop();
  }
  if (intakeTrue){
   Intake.spin(fwd, intakePct, pct);
   Intake.spin(fwd, intakePct, pct);
  }
  if(Controller1.ButtonB.PRESSED) {
    outTakeTrue = !outTakeTrue;
    Intake.stop();
    Intake.stop();
  }
  if (outTakeTrue){
    Intake.spin(reverse, 100, pct);
    Intake.spin(reverse, 100, pct);
    Controller1.rumble(".");
  }
  else {
    Intake.setStopping(coast);
    Intake.setStopping(coast);
  }
}

// SHOOTING

int catapultSpeed = 100;
int catapultTimer = 50;

bool hitLimit;

void catapultControl(){
  Catapult.setVelocity(100, pct);
  if (Controller1.ButtonL1.PRESSED){
    catapultTimer = 0;
    hitLimit = false;
    Catapult.setBrake(coast);
    Catapult.spinFor(reverse, 400, msec);
    PistonBoostL.set(true);
    PistonBoostR.set(true);
  }
  if (!hitLimit && catapultTimer >= 50){
    Catapult.setBrake(brake);
    Catapult.spin(reverse);
    PistonBoostL.set(false);
    PistonBoostR.set(false);
  }
  else {
    if (Limit.PRESSED){
      hitLimit = true;
      catapultTimer = 50;
      Catapult.stop();
    }
    catapultTimer++;
    Catapult.setBrake(brake);
    Catapult.stop(brake);
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


void tankDrive(){
  float maxSpeed = 100;

  int exp = 3;

  float leftPct = (Controller1.Axis3.position());
  float rightPct = (Controller1.Axis2.position());

  float leftNewPct = pow(leftPct, exp)/pow(maxSpeed, exp-1);
  float rightNewPct = pow(rightPct, exp)/pow(maxSpeed, exp-1);

  FL.spin(fwd, leftNewPct, pct);
  BL.spin(fwd, leftNewPct, pct);
  ML.spin(fwd, leftNewPct, pct);
  FR.spin(fwd, rightNewPct, pct);
  BR.spin(fwd, rightNewPct, pct);
  MR.spin(fwd, rightNewPct, pct);
}

void arcadeDrive(){
  float arcadeLeftPct = (Controller1.Axis3.position() + Controller1.Axis1.position() * 0.6);
  float arcadeRightPct = (Controller1.Axis3.position() - Controller1.Axis1.position()* 0.6);

  FL.spin(fwd, arcadeLeftPct * 1, pct);
  BL.spin(fwd, arcadeLeftPct * 1, pct);
  FR.spin(fwd, arcadeRightPct * 1, pct);
  BR.spin(fwd, arcadeRightPct * 1, pct);
}

void usercontrol(void) {
  Endgame.set(false);

  Left.resetRotation();
  Side.resetRotation();

  Endgame.set(false);
  PistonBoostL.set(false);
  PistonBoostR.set(false);

  FL.setBrake(brakeType::coast);
  FR.setBrake(brakeType::coast);
  BL.setBrake(brakeType::coast);
  BR.setBrake(brakeType::coast);
  
  // task odometryTask(positionTracking);
  // task drawFieldTask(drawField);
  // task chassisControlTask(chassisControl);

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
    catapultControl();
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
