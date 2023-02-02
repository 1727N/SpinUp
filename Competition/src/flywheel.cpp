#include "flywheel.h"
#include <iostream>

// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED           20

bool DRIVER_CONTROL;

// Maximum power we want to send to the flywheel motors
int FW_MAX_POWER    =        100;
double flywheelVoltage = 7;

// encoder counts per revolution depending on motor
#define MOTOR_TPR_TURBO         261.333
#define MOTOR_TPR_STANDARD      627.2
#define MOTOR_TPR_QUAD          360.0

// encoder tick per revolution
float           ticks_per_rev;          ///< encoder ticks per revolution

// Encoder
long            encoder_countsR;         ///< current encoder count
long            encoder_counts_lastR;    ///< current encoder count
long            encoder_countsL;         
long            encoder_counts_lastL;   

// velocity measurement
float           motor_velocityR;         ///< current velocity in rpm
float           motor_velocityL;         
long            nSysTime_lastR;          ///< Time of last velocity calculation
long            nSysTime_lastL;          

// TBH control algorithm variables
long            target_velocityR;        ///< target_velocity velocity
long            target_velocityL;        
float           current_errorR;          ///< error between actual and target_velocity velocities
float           current_errorL;          ///< error between actual and target_velocity velocities
float           last_errorR;             ///< error last time update called
float           last_errorL;             
float           gain;                   ///< gain
float           driveR;                  ///< final drive out of TBH (0.0 to 1.0) RIGHT
float           driveL;                  /// LEFT
float           drive_at_zeroR;          ///< drive at last zero crossing RIGHT
float           drive_at_zeroL;          ///LEFT
long            first_crossR;            ///< flag indicating first zero crossing RIGHT
long            first_crossL;            /// LEFT
float           drive_approxR;           ///< estimated open loop drive RIGHT
float           drive_approxL;           ///LEFT

// final motor drive
long            motor_driveR;            ///< final motor control value RIGHT
long            motor_driveL;            /// LEFT

/*Set the flywheen motors RIGHT  */
void
FwMotorSetR( int valueR )
{
  FlyFront.spin(fwd, valueR, pct);
}

/*Set the flywheen motors LEFT */
void
FwMotorSetL( int valueL )
{
	FlyBack.spin(fwd, valueL, pct);
}


/*Get the flywheen motor encoder count RIGHT*/
long
FwMotorEncoderGetR()
{
	return( FlyFront.position(rev));
}


/*Get the flywheen motor encoder count LEFT */
long
FwMotorEncoderGetL()
{
	return( FlyBack.position(rev));
}


/*Set the controller position RIGHT */
void
FwVelocitySetR( int velocityR, float predicted_driveR )
{
	// set target_velocity velocity (motor rpm)
	target_velocityR = velocityR;

	// Set error so zero crossing is correctly detected
	current_errorR = target_velocityR - motor_velocityR;
	last_errorR    = current_errorR;

	// Set predicted open loop drive value
	drive_approxR  = predicted_driveR;
	// Set flag to detect first zero crossing
	first_crossR   = 1;
	// clear tbh variable
	drive_at_zeroR = 0;
}


/*Set the controller position LEFT  */
void
FwVelocitySetL( int velocityL, float predicted_driveL )
{
	// set target_velocity velocity (motor rpm)
	target_velocityL = velocityL;

	// Set error so zero crossing is correctly detected
	current_errorL = target_velocityL - motor_velocityL;
	last_errorL    = current_errorL;

	// Set predicted open loop drive value
	drive_approxL  = predicted_driveL;
	// Set flag to detect first zero crossing
	first_crossL   = 1;
	// clear tbh variable
	drive_at_zeroL = 0;
}

void FwVelocitySet( int velocity, float predicted_drive){
  FwVelocitySetL( velocity, predicted_drive );
  FwVelocitySetR( velocity, predicted_drive );
}

/*Calculate the current flywheel motor velocity*/
void
FwCalculateSpeed()
{
	int     delta_msR;
	int     delta_msL;
	int     delta_encR;
	int     delta_encL;

	// Get current encoder value
	encoder_countsR = FwMotorEncoderGetR();
	encoder_countsL = FwMotorEncoderGetL();

	// This is just used so we don't need to know how often we are called
	// how many mS since we were last here
	delta_msR = vex::timer::system() - nSysTime_lastR;
	nSysTime_lastR = vex::timer::system();
	delta_msL = vex::timer::system() - nSysTime_lastL;
	nSysTime_lastL = vex::timer::system();

	// Change in encoder count
	delta_encR = (encoder_countsR - encoder_counts_lastR);
	delta_encL = (encoder_countsL - encoder_counts_lastL);

	// save last position
	encoder_counts_lastR = encoder_countsR;
	encoder_counts_lastL = encoder_countsL;

	// Calculate velocity in rpm
	motor_velocityR = (1000.0 / delta_msR) * delta_encR * 60.0 / ticks_per_rev;
	motor_velocityL = (1000.0 / delta_msL) * delta_encL * 60.0 / ticks_per_rev;
}

/*Update the velocity tbh controller variables RIGHT*/
void
FwControlUpdateVelocityTbhR()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	current_errorR = target_velocityR - motor_velocityR;

	// Calculate new control value
	driveR =  driveR + (current_errorR * gain);

	// Clip to the range 0 - 1.
	// We are only going forwards
	if( driveR > 1 )
		driveR = 1;
	if( driveR < 0 )
		driveR = 0;

	// Check for zero crossing
	if( signbit(current_errorR) != signbit(last_errorR) ) {
		// First zero crossing after a new set velocity command
		if( first_crossR ) {
			// Set drive to the open loop approximation
			driveR = drive_approxR;
			first_crossR = 0;
		}
		else
			driveR = 0.5 * ( driveR + drive_at_zeroR );

		// Save this drive value in the "tbh" variable
		drive_at_zeroR = driveR;
	}

	// Save last error
	last_errorR = current_errorR;
}


/*Update the velocity tbh controller variables */
void
FwControlUpdateVelocityTbhL()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	current_errorL = target_velocityL - motor_velocityL;

	// Calculate new control value
	driveL =  driveL + (current_errorL * gain);

	// Clip to the range 0 - 1.
	// We are only going forwards
	if( driveL > 1 )
		driveL = 1;
	if( driveL < 0 )
		driveL = 0;

	// Check for zero crossing
	if( signbit(current_errorL) != signbit(last_errorL) ) {
		// First zero crossing after a new set velocity command
		if( first_crossL ) {
			// Set drive to the open loop approximation
			driveL = drive_approxL;
			first_crossL = 0;
		}
		else
			driveL = 0.5 * ( driveL + drive_at_zeroL );

		// Save this drive value in the "tbh" variable
		drive_at_zeroL = driveL;
	}

	// Save last error
	last_errorL = current_errorL;
}

bool flywheelSpin;
int setVoltage = 10;

int fwTask(){
  while(1){
    if (flywheelSpin){
      FlyFront.spin(fwd, setVoltage, volt);
      FlyBack.spin(fwd, setVoltage, volt);
    }
    else {
    FlyFront.setStopping(coast);
    FlyBack.setStopping(coast);
    FlyFront.stop();
    FlyBack.stop();
    }
  }
}

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

double targetVelocity = 450;

void flyWheelPID(){
  double currentRPM = -(FlyFront.velocity(rpm) + FlyBack.velocity(rpm))/2;

  flyError = targetVelocity - currentRPM;

  if (flyError > 200){
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

  flyPowerPID = iteratedPID * 12/600;

  if (flyPowerPID < 0){
    flyPowerPID = 0;
  }

  if (flyPowerPID > 12){
    flyPowerPID = 12;
  }

  //https://www.vexforum.com/t/flywheel-velocity-pid/56914/12
}

int loopCount;

/*Task to control the velocity of the flywheel */
int FwControlTask()
{
	// Set the gain
	gain = 0.00025;

	// Set the encoder ticks per revolution
	ticks_per_rev = MOTOR_TPR_TURBO;

	while(1)
	{
    loopCount++;
    // if (Controller1.ButtonX.PRESSED){
    //   flykP += 1;
    // }
    // if (Controller1.ButtonA.PRESSED){
    //   flykP = flykP - 1;
    // }
    // if (Controller1.ButtonX.PRESSED){
    //   flykD += 0.001;
    // }
    if (loopCount == 10000){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print(FlyFront.velocity(rpm));

      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print(flykP);

      Controller1.Screen.setCursor(2,7);
      Controller1.Screen.print(flykI);

      Controller1.Screen.setCursor(2,13);
      Controller1.Screen.print(flykD);

      std::cout << "p " << flykP << "i " << flykI << "d" << flykD << std::endl << std::endl;
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
      // flyWheelPID();
      // flyPowerPID = flywheelVoltage;
      // FlyFront.spin(reverse, flyPowerPID, volt);
      // FlyBack.spin(reverse, flyPowerPID, volt);

    FwCalculateSpeed();

		// Do the velocity TBH calculations
		FwControlUpdateVelocityTbhR() ;
		FwControlUpdateVelocityTbhL() ;

		// Scale drive into the range the motors need
		motor_driveR  = (driveR * FW_MAX_POWER) + 0.5;
		motor_driveL  = (driveL * FW_MAX_POWER) + 0.5;

		// Final Limit of motor values - don't really need this
		if( motor_driveR >  100 ) motor_driveR =  100;
		if( motor_driveR < -100 ) motor_driveR = -100;
		if( motor_driveL >  100 ) motor_driveL =  100;
		if( motor_driveL < -100 ) motor_driveL = -100;

		// and finally set the motor control value
		// FwMotorSetR( motor_driveR );
		// FwMotorSetL( motor_driveL );

    }
		// Calculate velocity
	}
}