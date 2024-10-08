#include "drivetrain.h"
#include <iostream>

double xTargetLocation = xPosGlobal;
double yTargetLocation = yPosGlobal;
double targetFacingAngle = 0;

int driveTimer;

double xDistToTarget = 0;
double yDistToTarget = 0;

// Angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

double drivePowerFLBR = 0;
double drivePowerFRBL = 0;

bool runChassisControl = false;

bool onlyTurn = false;
bool directDriveOn;

int timeOutValue = 2500;

double maxAllowedSpeed = 1.0;

#define toRadians M_PI/180
#define toDegrees 180/M_PI

/*---------------------------------------------------------- CONTROL METHODS ----------------------------------------------------------*/

// ODOM DRIVE
void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength = 2500, double maxSpeed = 1.0) {
  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  targetFacingAngle = currentAbsoluteOrientation;

  runChassisControl = true;
  onlyTurn = false;
  directDriveOn = false;

  timeOutValue = timeOutLength;
  Brain.resetTimer();
  maxAllowedSpeed = maxSpeed;
}

// ODOM TURN
void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength = 1500, double maxSpeed = 1.0) {
  targetFacingAngle = atan2(yCoordToFace - yPosGlobal, xCoordToFace - xPosGlobal);

  if(targetFacingAngle < 0) {
    targetFacingAngle = 2 * M_PI - fabs(targetFacingAngle);
  }
  xTargetLocation = xPosGlobal;
  yTargetLocation = yPosGlobal;

  runChassisControl = true;
  onlyTurn = true;
  directDriveOn = false;

  timeOutValue = timeOutLength;

  Brain.resetTimer();

  maxAllowedSpeed = maxSpeed;
}

double setPoint;
double currentPoint;

// USE THIS METHOD FOR NO ODOM AUTON
void directDrive(double xDist, double timeOutLength, double maxSpeed = 1.0){
  Left.setPosition(0, rev);
  setPoint = xDist;
  targetFacingAngle = currentAbsoluteOrientation;
  currentPoint = 0;

  runChassisControl = true;
  onlyTurn = false;
  directDriveOn = true;

  timeOutValue = timeOutLength;
  Brain.resetTimer();
  maxAllowedSpeed = maxSpeed;
}

// USE THIS METHOD FOR NO ODOM AUTON
void turnTo(double targetAngle, double timeOutLength = 2500) {
  targetFacingAngle = targetAngle * toRadians;

  if (fabs(targetFacingAngle - currentAbsoluteOrientation) > M_PI_2){
    // turnkI = 5;    
  }
  else {
    // turnkI = 7;
  }

  xTargetLocation = xPosGlobal;
  yTargetLocation = yPosGlobal;

  runChassisControl = true;
  onlyTurn = true;
  directDriveOn = false;

  timeOutValue = timeOutLength;

  Brain.resetTimer();
}

// DOES NOT FUNCTION CORRECTLY: GOING BACKWARDS DOES NOT WORK IN ODOM
void setDrivePower(double theta) {
  if (onlyTurn || directDriveOn){
    drivePowerFLBR = 1;
    drivePowerFRBL = 1;
    return;
  }

  if (theta > M_PI_2 && theta < M_PI_2*3){
    drivePowerFLBR = -1;
    drivePowerFRBL = -1;
  }
  else if (theta < -M_PI_2 && theta > -M_PI_2*3){
    drivePowerFLBR = -1;
    drivePowerFRBL = -1;
  }
  else {
    drivePowerFLBR = 1;
    drivePowerFRBL = 1;
  }
}

/*---------------------------------------------------------- DRIVE PID ----------------------------------------------------------*/

double driveError = 0;
double drivePrevError = 0;

double driveMaxError = 0.1;

double driveIntegral = 0;
double driveIntegralBound = 1.5;

double driveDerivative = 0;

double drivekP = 1.2;
double drivekI = 0.1;
double drivekD = 4;

double drivePowerPID = 0;

bool isPositive;

void drivePID() {
  if (!directDriveOn){
    driveError = sqrt(pow((xPosGlobal - xTargetLocation), 2) + pow((yPosGlobal - yTargetLocation), 2));
  }
  else {
    driveError = setPoint - currentPoint;
    drivePowerFLBR = 1;
    drivePowerFRBL = 1;
  }
  
  // Only integrate when within bounds
  if(fabs(driveError) < driveIntegralBound) {
    driveIntegral += driveError;
  }
  else {
    driveIntegral = 0;
  }

  // Reset after setpoint to stop integral windup
  if(driveError * drivePrevError < 0) {
    driveIntegral = 0;
  } 

  driveDerivative = driveError - drivePrevError;

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  if(drivePowerPID > 12) {
    drivePowerPID = 12;
  }

  if(fabs(driveError) < driveMaxError) {
    drivePowerPID = 0;
  }
}

/*---------------------------------------------------------- TURN PID ----------------------------------------------------------*/

double turnError = 0;
double turnPrevError = 0;

double turnMaxError = 0.003;

double turnIntegral = 0;
double turnIntegralBound = 0.09;

double turnDerivative = 0;

double turnkP = 13;
double turnkI = 3.5;
double turnkD = 50;


double turnPowerPID = 0;

void turnPID() {
  turnError = currentAbsoluteOrientation - targetFacingAngle;

  if(fabs(turnError) > M_PI) {
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * M_PI - turnError);
  }

  // Integrate when within bounds
  if(fabs(turnError) < turnIntegralBound) {
    turnIntegral += turnError;
  }
  else {
    turnIntegral = 0;
  }

  // Prevent windup
  if(turnError * turnPrevError < 0) {
    turnIntegral = 0;
  } 

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPowerPID = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  if (!onlyTurn){
    if(turnPowerPID > 12) {
      turnPowerPID = 12;
    }
  }
  else {
    if(turnPowerPID > 12) {
      turnPowerPID = 12;
    }
    drivePowerFLBR = 0;
    drivePowerFRBL = 0;
  }

  // if (turnPowerPID < 1 && turnPowerPID > 0){
  //   turnPowerPID = 1;
  // }
  // else if (turnPowerPID > -1 && turnPowerPID < 0){
  //   turnPowerPID  = -1;
  // }

  if(fabs(turnError) < turnMaxError) {
    turnPowerPID = 0;
  }

}

double FrontLeftPower = 0;
double FrontRightPower = 0;
double BackLeftPower = 0;
double BackRightPower = 0;

/*---------------------------------------------------------- CHASSIS CONTROL TASK ----------------------------------------------------------*/

int restTime;
int maxRestTime = 5;

int chassisControl() {
  while(1) {
    if(runChassisControl) {
      if (directDriveOn){
        currentPoint = Left.position(rev) * 2.75 * M_PI;
      }
      else {
        xDistToTarget = xTargetLocation - xPosGlobal;
        yDistToTarget = yTargetLocation - yPosGlobal;

        hypotenuseAngle = atan2(yDistToTarget, xDistToTarget);

        if(hypotenuseAngle < 0) {
          hypotenuseAngle += 2 * M_PI;
        }

        // The angle the robot needs to travel relative to its forward direction in order to go toward the target
        robotRelativeAngle = hypotenuseAngle - currentAbsoluteOrientation;

        if(robotRelativeAngle > 2 * M_PI) {
          robotRelativeAngle -= 2 * M_PI;
        }
        else if(robotRelativeAngle < 0) {
          robotRelativeAngle += 2 * M_PI;
        }
      }

      //setDrivePower(robotRelativeAngle);

      drivePID();
      turnPID();

      FrontLeftPower = (turnPowerPID + (drivePowerFLBR * drivePowerPID)) * maxAllowedSpeed;
      FrontRightPower = ((drivePowerFRBL * drivePowerPID) - turnPowerPID) * maxAllowedSpeed;
      BackLeftPower = ((drivePowerFRBL * drivePowerPID) + turnPowerPID) * maxAllowedSpeed;
      BackRightPower = ((drivePowerFLBR * drivePowerPID) - turnPowerPID) * maxAllowedSpeed;
      
      FL.spin(directionType::fwd, FrontLeftPower, voltageUnits::volt);
      FR.spin(directionType::fwd, FrontRightPower, voltageUnits::volt);
      ML.spin(directionType::fwd, FrontLeftPower, voltageUnits::volt);
      MR.spin(directionType::fwd, FrontRightPower, voltageUnits::volt);
      BL.spin(directionType::fwd, BackLeftPower, voltageUnits::volt);
      BR.spin(directionType::fwd, BackRightPower, voltageUnits::volt);
      
      
      if (onlyTurn){
        if (fabs(turnError) < turnMaxError){
          restTime++;
        }
        else {
          restTime = 0;
        }
        if (restTime >= maxRestTime){
          runChassisControl = false;
          std::cout << "<------------------ REACHED SETPOINT: " << Brain.timer(timeUnits::msec) << std::endl << std::endl;
        }
      }

      else {
        if (fabs(driveError) < driveMaxError){
          restTime++;
        }
        else {
          restTime = 0;
        }
        if(restTime >= maxRestTime) {
          runChassisControl = false; 
          std::cout << "<------------------ REACHED SETPOINT" << std::endl << std::endl;
        }
      }

      if(Brain.timer(timeUnits::msec) > timeOutValue) {
        runChassisControl = false;
        std::cout << "TIMED OUT ------------------>" << std::endl << std::endl;
      }

      std::cout << "Rest: " << restTime << std::endl;
      std::cout << "Drive Error: " << driveError << std::endl;
      std::cout << "Turn Error: " << (turnError * toDegrees) << std::endl;
      std::cout << "Drive Power: " << drivePowerPID << std::endl;
      std::cout << "Turn Power: " << turnPowerPID << std::endl << std::endl;
    }
    else {
      FR.setBrake(brake);
      FL.setBrake(brake);
      ML.setBrake(brake);
      MR.setBrake(brake);
      BR.setBrake(brake);
      BL.setBrake(brake);
      FR.stop();
      FL.stop();
      BR.stop();
      BL.stop();
    }
    task::sleep(20);
  }
  return 1;
}