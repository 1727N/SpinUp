#include "flywheel.h"
#include <iostream>
#include "chassis-control.h"

#define FW_LOOP_SPEED   20

bool DRIVER_CONTROL;

// Maximum power we want to send to the flywheel motors
double FW_MAX_POWER = 12;
double flywheelVoltage = 8;

// velocity measurement
float           motor_velocity;

// TBH control algorithm variables
long            target_velocity;        ///< target_velocity velocity
float           current_error;          ///< error between actual and target_velocity velocities
float           last_error;             ///< error last time update called
float           gain;                   ///< gain
float           driveR;                  ///< final drive out of TBH (0.0 to 1.0) RIGHT
float           drive_at_zero;          ///< drive at last zero crossing RIGHT
long            first_cross;            ///< flag indicating first zero crossing RIGHT
float           drive_approx;           ///< estimated open loop drive RIGHT

// final motor drive
long            motor_drive;            ///< final motor control value RIGHT

/*Set the flywheen motors RIGHT  */
void
FwMotorSet( double valueR )
{
  FlyFront.spin(reverse, valueR, volt);
  FlyBack.spin(reverse, valueR, volt);
}


void FwVelocitySet( int FWvelocity, float predicted_drive){
  // set target_velocity velocity (rpm)
	target_velocity = FWvelocity;

	// Set error so zero crossing is correctly detected
	current_error = target_velocity - motor_velocity;
	last_error    = current_error;

	// Set predicted open loop drive value
	drive_approx  = predicted_drive;
	// Set flag to detect first zero crossing
	first_cross   = 1;
	// clear tbh variable
	drive_at_zero = 0;
}

/*Calculate the current flywheel motor velocity*/
void
FwCalculateSpeed()
{
  motor_velocity = -Flywheel.velocity(rpm);
}

/*Update the velocity tbh controller variables*/
void
FwControlUpdateVelocityTbh()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	current_error = target_velocity - motor_velocity;

	// Calculate new control value
	driveR =  driveR + (current_error * gain);

	// Clip to the range 0 - 1.
	// We are only going forwards
	if( driveR > 1 )
		driveR = 1;
	if( driveR < 0 )
		driveR = 0;

	// Check for zero crossing
	if( signbit(current_error) != signbit(last_error) ) {
		// First zero crossing after a new set velocity command
		if( first_cross ) {
			// Set drive to the open loop approximation
			driveR = drive_approx;
			first_cross = 0;
		}
		else
			driveR = 0.5 * ( driveR + drive_at_zero );

		// Save this drive value in the "tbh" variable
		drive_at_zero = driveR;
	}

	// Save last error
	last_error = current_error;
}

/*---------------------------------------------------------- PID CONTROL ----------------------------------------------------------*/

bool flyWheelOn = false;

double flyError = 0;
double flyPrevError = 0;

double flyMaxError = 5;
double flyIntegral = 0;
double flyIntegralBound = 1;

double flyDerivative = 0;

double flykP = 5;
double flykI = 0;
double flykD = 0;

double flyPowerPID = 0;

double targetVelocity = 2000;

void flyWheelPID(){
  double currentRPM = -Flywheel.velocity(rpm);

  flyError = targetVelocity - currentRPM;

  if (flyError > 500){
    flyPowerPID = 12;
    return;
  }

  if(fabs(flyError) < flyIntegralBound) {
    flyIntegral += flyError;
  }
  else {
    flyIntegral = 0;
  }

  if(flyError * flyPrevError < 0) {
    flyIntegral = 0;
  } 

  flyDerivative = flyError - flyPrevError;

  flyPrevError = flyError;

  double iteratedPID = (flyError * flykP + flyIntegral * flykI + flyDerivative * flykD);

  flyPowerPID = iteratedPID * 12/targetVelocity;

  if (flyPowerPID < 0){
    flyPowerPID = 0;
  }

  if (flyPowerPID > 12){
    flyPowerPID = 12;
  }
}

/*---------------------------------------------------------- FLYWHEEL TASK ----------------------------------------------------------*/

int loopCount;

int FwControlTask()
{
	gain = 0.00005;

	while(1)
	{
    loopCount++;
    // if (Controller1.ButtonX.PRESSED){
    //   flykP += .1;
    // }
    // if (Controller1.ButtonA.PRESSED){
    //   flykP -= .1;
    // }
    // if (Controller1.ButtonY.PRESSED){
    //   flykD += 0.1;
    // }
    // if (Controller1.ButtonB.PRESSED){
    //   flykD -= 0.1;
    // }
    if (loopCount == 10000){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print(Flywheel.velocity(rpm));

      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print(flykP);

      Controller1.Screen.setCursor(2,7);
      Controller1.Screen.print(flykI);

      Controller1.Screen.setCursor(2,13);
      Controller1.Screen.print(flykD);

      loopCount = 0;
    }
    if (DRIVER_CONTROL){
      if (flyWheelOn){
        flyWheelPID();
        flyPowerPID = flywheelVoltage;
        FlyFront.spin(reverse, flyPowerPID, volt);
        FlyBack.spin(reverse, flyPowerPID, volt);
        
        //FlyFront.spin(reverse, 400, rpm);
        //FlyBack.spin(reverse, 400, rpm);
      }
      else {
        FlyFront.stop(coast);
        FlyBack.stop(coast);
      }
    }
    else {
    FwCalculateSpeed();

		// Do the velocity TBH calculations
		FwControlUpdateVelocityTbh() ;

		// Scale drive into the range the motors need
		motor_drive = (driveR * FW_MAX_POWER) + 0.5;

		// Power clipping
		if( motor_drive >  FW_MAX_POWER ) motor_drive =  FW_MAX_POWER;
		if( motor_drive < -FW_MAX_POWER ) motor_drive = -FW_MAX_POWER;
		if( motor_drive >  FW_MAX_POWER ) motor_drive =  FW_MAX_POWER;
		if( motor_drive < -FW_MAX_POWER ) motor_drive = -FW_MAX_POWER;

		// and finally set the motor control value  
		FwMotorSet( motor_drive );

    // std::cout << Flywheel.velocity(rpm) << std::endl;
    // std::cout << current_error << std::endl;
    // std::cout << motor_drive << std::endl << std::endl;
    }
    task::sleep(10);
	}
}