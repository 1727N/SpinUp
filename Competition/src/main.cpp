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
// FlyFront             motor         5               
// FlyBack              motor         6               
// Vision               vision        3               
// Intake               motor         16              
// Double1              digital_out   C               
// Endgame              digital_out   B               
// Indexer              motor         15              
// Pressure             digital_out   A               
// Side                 rotation      2               
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
// FlyFront             motor         5               
// FlyBack              motor         6               
// Vision               vision        3               
// Intake               motor         16              
// Double1              digital_out   C               
// Endgame              digital_out   B               
// Indexer              motor         15              
// Pressure             digital_out   A               
// IntakePump           digital_out   D               
// Side                 rotation      2               
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
// FlyFront             motor         5               
// FlyBack              motor         6               
// Vision               vision        3               
// Intake               motor         16              
// Double1              digital_out   C               
// Endgame              digital_out   B               
// Indexer              motor         15              
// Pressure             digital_out   A               
// IntakePump           digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----
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
// Double1              digital_out   C               
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

  wait(800, msec);

  driverControl = false;
}

void driveForDist(double xDist, int timeOutLength, double maxSpeed){
  directDrive(xDist, timeOutLength, maxSpeed);
  waitUntil(runChassisControl == false);
}

void turnToAngle(int targetAngle, int timeOutLength){
  turnTo(targetAngle, timeOutLength);
  waitUntil(runChassisControl == false);
}

void rollFor(int timeRoll){
  Intake.spin(reverse, 60, pct);
  wait(timeRoll, msec);
  Intake.stop();
}

/* -------------------------- AUTON PROGRAMS -------------------------- */

//SKILLS
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
void shoot(){
  //if (FlyFront.velocity(rpm) < -440 && FlyFront.velocity(rpm) > -460){
  Indexer.setVelocity(100, pct);
  Intake.setVelocity(90, pct);
  Indexer.spin(reverse);
  Intake.spin(fwd);
  Double1.set(true);
  Pressure.set(true);
  wait(600, msec);
  Pressure.set(false);
  Double1.set(false);
  Indexer.stop();
  Intake.stop();
  //}
}

void rollerStart(){
  THETA_START = 0;

  driveForDist(-4.5, 800, 1);

  Indexer.spin(reverse, 100, pct);
  wait(400, msec);
  Indexer.stop();

  driveForDist(8, 1300, 1);

  turnToAngle(317, 1000);

  // FW_MAX_POWER = 80;
  // FwVelocitySet( 70, 1 );
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

  turnToAngle(315, 1000);

  // FW_MAX_POWER = 80;
  // FwVelocitySet( 70, 1 );
  FlyFront.spin(reverse, 460, rpm);
  FlyBack.spin(reverse, 460, rpm);

  driveForDist(63, 2000, 1);

  turnToAngle(40, 1000);
  shoot();

  Intake.spin(fwd, 100, pct);
  Indexer.spin(fwd, 100, pct);
  wait(800, msec);

  Intake.stop();
  Indexer.stop();

  shoot();

  turnToAngle(133, 1000);

  driveForDist(-59, 1700, 1);

  turnToAngle(90, 800);

  driveForDist(-15, 1300, 1);

  Indexer.spin(reverse, 100, pct);
  wait(700, msec);
  Indexer.stop();
}

void twopointer(){
  driveForDist(-20, 3000, 0.6);
  driveForDist(20, 3000, 0.6);
}

void tuning(){
  twopointer();
  //soloAWP();
  //rollerStart();
  //driveForDist(10, 5000, 1);
  //driveForDist(-10, 5000, 1);
  //turnTo(90, 4000);

}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  //reset sensors
  Left.resetRotation();
  //Right.resetRotation();
  Side.resetPosition();

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

  //autonSkills();
  tuning();
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

void puncherControl(){
  if (Controller1.ButtonUp.PRESSED){
    Indexer.setVelocity(100, pct);
    Intake.setVelocity(90, pct);
    
    if (flyWheelOn && -FlyFront.velocity(rpm) > 270)
    {
      Indexer.spin(reverse);
      Intake.spin(fwd);
      flywheelVoltage = 10.2;
      Double1.set(true);
      Pressure.set(true);
      wait(2000, msec);
      Pressure.set(false);
      Double1.set(false);
      Indexer.stop();
      Intake.stop();
      flywheelVoltage = 8;
    }
    else {  
      flyWheelOn = true; 
    }
  }
}

void catapultControl(){
  if (Controller1.ButtonY.pressing() && Controller1.ButtonRight.pressing()){  
    Endgame.set(true);
  }
}

void rollerControl(){
  if (Controller1.ButtonA.pressing()){
    Indexer.spin(fwd, 40, pct);
  }
}

bool pump;

void usercontrol(void) {
  //flywheelTask.stop();

  //https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/35

  DRIVER_CONTROL = true;
  driverControl = true;

  Endgame.set(false);

  Left.resetRotation();

  FL.resetRotation();
  FR.resetRotation();
  BL.resetRotation();
  BR.resetRotation();

  FL.setBrake(brakeType::coast);
  FR.setBrake(brakeType::coast);
  BL.setBrake(brakeType::coast);
  BR.setBrake(brakeType::coast);
  
  odometryTask.stop();
  drawFieldTask.stop();
  chassisControlTask.stop();
  
  task odometryTask(positionTracking);
  task drawFieldTask(drawField);
  //task chassisControlTask(chassisControl);
  task flywheelTask(FwControlTask);

  FlyFront.setBrake(coast);
  FlyBack.setBrake(coast);

  pump = false;

  while (1) {
    float maxSpeed = 100;
    // float leftPct = (Controller1.Axis3.position())/maxSpeed;
    // float rightPct = (Controller1.Axis2.position())/maxSpeed;

    // float leftNewPct = leftPct * leftPct * leftPct * 100;
    // float rightNewPct = rightPct * rightPct * rightPct * 100;

    int exp = 3;

    float leftPct = (Controller1.Axis3.position());
    float rightPct = (Controller1.Axis2.position());

    float arcadeLeftPct = (Controller1.Axis3.position() + Controller1.Axis1.position());
    float arcadeRightPct = (Controller1.Axis3.position() - Controller1.Axis1.position());

    float leftNewPct = pow(leftPct, exp)/pow(maxSpeed, exp-1);
    float rightNewPct = pow(rightPct, exp)/pow(maxSpeed, exp-1);

    float arcadeLeftNewPct = pow(arcadeLeftPct, exp)/pow(maxSpeed, exp-1);
    float arcadeRightNewPct = pow(arcadeRightPct, exp)/pow(maxSpeed, exp-1);


    FL.spin(fwd, leftNewPct, pct);
    BL.spin(fwd, leftNewPct, pct);
    FR.spin(fwd, rightNewPct, pct);
    BR.spin(fwd, rightNewPct, pct);

    // FL.spin(fwd, arcadeLeftNewPct, pct);
    // BL.spin(fwd, arcadeLeftNewPct, pct);
    // FR.spin(fwd, arcadeRightNewPct, pct);
    // BR.spin(fwd, arcadeRightNewPct, pct);

    intakeControl();
    puncherControl();
    catapultControl();
    rollerControl();

    if (flyWheelOn){
      FL.setBrake(brake);
      FR.setBrake(brake);
      BR.setBrake(brake);
      BL.setBrake(brake);
    }
    else {
      FL.setBrake(coast);
      FR.setBrake(coast);
      BR.setBrake(coast);
      BL.setBrake(coast);
    }

    if (Controller1.ButtonL1.PRESSED){
      flyWheelOn = true;
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
  //Competition.bStopAllTasksBetweenModes = true;
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
