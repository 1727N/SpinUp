#include "chassis-control.h"
#include <iostream>

//target coords
double xTargetLocation = xPosGlobal;
double yTargetLocation = yPosGlobal;
double targetFacingAngle = 0;

//distances
double xDistToTarget = 0;
double yDistToTarget = 0;

//angle of hypotenuse of X and Y distances
double hypotenuseAngle = 0;

double robotRelativeAngle = 0;

//front left + back right drive power
double drivePowerFLBR = 0;
//front right + back left drive power
double drivePowerFRBL = 0;

bool runChassisControl = false;

bool onlyTurn = false;
bool directDriveOn;

int timeOutValue = 2500;

double maxAllowedSpeed = 1.0;

#define toRadians M_PI/180;

//Sets the target position and indicates a specific target heading
void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength = 2500, double maxSpeed = 1.0) {
  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  targetFacingAngle = targetAngle * toRadians;

  runChassisControl = true;
  onlyTurn = false;
  directDriveOn = false;

  timeOutValue = timeOutLength;
  Brain.resetTimer();
  maxAllowedSpeed = maxSpeed;
}

double setPoint;
double currentPoint;

void directDrive(double xDist, double timeOutLength, double maxSpeed = 1.0){
  Left.resetRotation();
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

// turns toward a specific heading

void turnTo(double targetAngle, double timeOutLength = 2500) {
  targetFacingAngle = targetAngle * toRadians;

  xTargetLocation = xPosGlobal;
  yTargetLocation = yPosGlobal;

  runChassisControl = true;
  onlyTurn = true;
  directDriveOn = false;

  timeOutValue = timeOutLength;

  Brain.resetTimer();
}

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

/* 
  Sets the drive power for each set of opposing corners. 
  input: The angle of the target relative to the robot's "forward"
  result: Sets each value to a decimal from 0.0 to 1.0 representing 0% to 100% motor power
*/
void setDrivePower(double theta) {
  if (onlyTurn){
    drivePowerFLBR = 0;
    drivePowerFRBL = 0;
  }
  else {
    drivePowerFLBR = 1;
    drivePowerFRBL = 1;
  }
}

double driveError = 0;
double drivePrevError = 0;

double driveMaxError = 0.1;

double driveIntegral = 0;
double driveIntegralBound = 1.5;

double driveDerivative = 0;

double drivekP = 10;
double drivekI = 0;
double drivekD = 0;

double drivePowerPID = 0;

bool isPositive;

void drivePID() {
  //Error is equal to the total distance away from the target (uses distance formula with current position and target location)
  if (!directDriveOn){
    driveError = sqrt(pow((xPosGlobal - xTargetLocation), 2) + pow((yPosGlobal - yTargetLocation), 2));
  }
  else {
    driveError = setPoint - currentPoint;
  }
  
  //use integral if close enough to target
  if(fabs(driveError) < driveIntegralBound) {
    driveIntegral += driveError;
  }
  else {
    driveIntegral = 0;
  }

  //reset integral if we pass the target
  if(driveError * drivePrevError < 0) {
    driveIntegral = 0;
  } 

  driveDerivative = driveError - drivePrevError;

  drivePrevError = driveError;

  drivePowerPID = (driveError * drivekP + driveIntegral * drivekI + driveDerivative * drivekD);

  //Limit power output to 12V
  if(drivePowerPID > 10) {
    drivePowerPID = 10;
  }

  if(fabs(driveError) < driveMaxError) {
    drivePowerPID = 0;
  }
}

double turnError = 0;
double turnPrevError = 0;

double turnMaxError = 0.01;

double turnIntegral = 0;
double turnIntegralBound = 0.09;

double turnDerivative = 0;

// double turnkP = 26;
// double turnkI = 0;
// double turnkD = 0;

double turnkP = 26;
double turnkI = 5;
double turnkD = 44;

double turnPowerPID = 0;

void turnPID() {

  //Error is equal to the difference between the current facing direction and the target direction
  turnError = currentAbsoluteOrientation - targetFacingAngle;

  if(fabs(turnError) > M_PI) {
    turnError = (turnError/fabs(turnError)) * -1 * fabs(2 * M_PI - turnError);
  }

  //use integral if close enough to target
  if(fabs(turnError) < turnIntegralBound) {
    turnIntegral += turnError;
  }
  else {
    turnIntegral = 0;
  }

  //reset integral if we pass the target
  if(turnError * turnPrevError < 0) {
    turnIntegral = 0;
  } 

  turnDerivative = turnError - turnPrevError;

  turnPrevError = turnError;

  turnPowerPID = (turnError * turnkP + turnIntegral * turnkI + turnDerivative * turnkD);

  //Limit power output to 12V
  if (!onlyTurn){
    if(turnPowerPID > 2) {
      turnPowerPID = 2;
    }
  }
  else {
    if(turnPowerPID > 12) {
      turnPowerPID = 12;
    }
  }
  

  if(fabs(turnError) < turnMaxError) {
    turnPowerPID = 0;
  }

}

/*
NOTES

Bottom left corner of field is (0,0)

Angles are like a unit circle, so the positive X direction is 0 and positive Y direction is pi/2

*/

double FrontLeftPower = 0;
double FrontRightPower = 0;
double BackLeftPower = 0;
double BackRightPower = 0;

/* CHASSIS CONTROL TASK */
int chassisControl() {
  while(1) {
    if(runChassisControl) {
      if (directDriveOn){
        currentPoint = -Left.rotation(rev) * 2.75 * M_PI;
      }
      else {
        xDistToTarget = xTargetLocation - xPosGlobal;
        yDistToTarget = yTargetLocation - yPosGlobal;

        hypotenuseAngle = atan2(yDistToTarget, xDistToTarget);

        if(hypotenuseAngle < 0) {
         hypotenuseAngle += 2 * M_PI;
        }

        //The angle the robot needs to travel relative to its forward direction in order to go toward the target
        robotRelativeAngle = hypotenuseAngle - currentAbsoluteOrientation + M_PI_2;

        if(robotRelativeAngle > 2 * M_PI) {
          robotRelativeAngle -= 2 * M_PI;
        }
        else if(robotRelativeAngle < 0) {
          robotRelativeAngle += 2 * M_PI;
        }
      }

      //Get the power percentage values for each set of motors
      setDrivePower(robotRelativeAngle);

      //get PID values for driving and turning
      drivePID();
      turnPID();

      //set power

      //TODO: CHANGE DRIVEPOWERFLBR, etc. TO 1 to test effects. May have to change moving to a point into a two step process.
      FrontLeftPower = (turnPowerPID + (drivePowerFLBR * drivePowerPID)) * maxAllowedSpeed;
      FrontRightPower = ((drivePowerFRBL * drivePowerPID) - turnPowerPID) * maxAllowedSpeed;
      BackLeftPower = ((drivePowerFRBL * drivePowerPID) + turnPowerPID) * maxAllowedSpeed;
      BackRightPower = ((drivePowerFLBR * drivePowerPID) - turnPowerPID) * maxAllowedSpeed;
      
      FL.spin(directionType::fwd, FrontLeftPower, voltageUnits::volt);
      FR.spin(directionType::fwd, FrontRightPower, voltageUnits::volt);
      BL.spin(directionType::fwd, BackLeftPower, voltageUnits::volt);
      BR.spin(directionType::fwd, BackRightPower, voltageUnits::volt);
      
      if (onlyTurn){
        if (fabs(turnError) < 0.003){
          runChassisControl = false;
          std::cout << "<------------------ REACHED SETPOINT" << std::endl << std::endl;
        }
      }

      else {
        if(fabs(driveError) < 0.1 && fabs(turnError) < 0.003) {
          runChassisControl = false; 
          std::cout << "<------------------ REACHED SETPOINT" << std::endl << std::endl;
        }
      }

      if(Brain.timer(timeUnits::msec) > timeOutValue) {
        runChassisControl = false;
        std::cout << "TIMED OUT ------------------>" << std::endl << std::endl;
      }

      std::cout << "Drive Error: " << driveError << std::endl;
      std::cout << "Turn Error: " << turnError << std::endl;
      std::cout << "Drive Power: " << drivePowerPID << std::endl;
      std::cout << "Turn Power: " << turnPowerPID << std::endl;
      std::cout << "FL: " << drivePowerFLBR << std::endl;
      std::cout << "FR: " << drivePowerFRBL << std::endl;
      std::cout << "FL Power: " << FrontLeftPower << std::endl;
      std::cout << "FR Power: " << FrontRightPower << std::endl;
      std::cout << "BL Power: " << BackLeftPower << std::endl;
      std::cout << "BR Power: " << BackRightPower << std::endl << std::endl;

      // Brain.Screen.setCursor(1,2);
      // Brain.Screen.print("Hyp angle: %f", hypotenuseAngle);
      // Brain.Screen.setCursor(2,2);
      // Brain.Screen.print(FrontLeftPower);
      //Brain.Screen.print("botRelativeAngle: %f", robotRelativeAngle);

      

      // Brain.Screen.setCursor(3,25);
      // Brain.Screen.print("drive error: %f", driveError);

      // Brain.Screen.setCursor(5,25);:endl;
      // Brain.Screen.print("turn error: %f", turnError);

      // Brain.Screen.setCursor(7,25);
      // Brain.Screen.print("turnPID power: %f", drivePowerPID);

      // Brain.Screen.setCursor(8,25);
      // Brain.Screen.print("FL Power: %f", FrontLeftPower);

      // Brain.Screen.setCursor(4,2);
      // Brain.Screen.print("absolute orientation: %f", currentAbsoluteOrientation);
      // // Brain.Screen.setCursor(3,2);
      // Brain.Screen.print(drivePowerFLBR);

    }
    //What to do when not using the chassis controls
    else {
      // FrontLeftDrive.stop(brakeType::brake);
      // FrontRightDrive.stop(brakeType::brake);
      // BackLeftDrive.stop(brakeType::brake);
      // BackRightDrive.stop(brakeType::brake);
      // FrontLeftDrive.stop(brakeType::coast);
      // FrontRightDrive.stop(brakeType::coast);
      // BackLeftDrive.stop(brakeType::coast);
      // BackRightDrive.stop(brakeType::coast);
    }
    
    task::sleep(20);

  }

  return 1;
}