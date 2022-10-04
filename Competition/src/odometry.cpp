#include "odometry.h"
#include <iostream>

//CONSTANTS / Hard-Coded Values
//Radius of tracking wheels in inches
double WHEEL_RADIUS = 1.379; //was 1.379 //1.43

//Starting angle (relative to field) (RADIANS)
double THETA_START = M_PI_2;

//The starting x and y coordinates of the bot (INCHES)
  //These distances are relative to some point (0,0) on the field
  //Relative to: BOTTOM LEFT CORNER
double X_START = 71.5; //19.1
double Y_START = 71.5; //8.5

//Distances of tracking wheels from tracking center (INCHES)
double LTrackRadius = 6.5; //was 6.87
double RTrackRadius = 6.5; //6.8335
double STrackRadius = 5.85;

//Calculated Values (every loop)
//Angles (DEGREES)
double LPos = 0;
double RPos = 0;
double SPos = 0;

double LPrevPos = 0;
double RPrevPos = 0;
double SPrevPos = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double deltaDistL = 0;
double deltaDistR = 0;
double deltaDistS = 0;

//Distance summations (since last reset)
double totalDeltaDistL = 0;
double totalDeltaDistR = 0;

//The current angle of the bot (RADIANS)
double currentAbsoluteOrientation = THETA_START;
//The previous angle of the bot (RADIANS)
double previousTheta = THETA_START;

//The change in Theta each loop (RADIANS)
double deltaTheta = 0;

//The Average angle Theta (In RADIANS) throughout the arc
  //currentAbsoluteOrientation + (deltaTheta / 2)
double avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

//The changes in the X and Y positions (INCHES)
/*These are calculated on a local basis each loop,
then converted to global position changes */
double deltaXLocal = 0;
double deltaYLocal = 0;

//The X and Y offsets converted from their local forms (INCHES)
double deltaXGlobal = 0;
double deltaYGlobal = 0;

//The global position of the bot (INCHES)
double xPosGlobal = X_START;
double yPosGlobal = Y_START;


int positionTracking() {
  std::cout << "Initial" << std::endl;
  std::cout << "x: " << xPosGlobal << std::endl;
  std::cout << "y: " << yPosGlobal << std::endl;
  std::cout << "heading: " << currentAbsoluteOrientation << std::endl << std::endl;

  while(1) {
    LPos = Left.position(rotationUnits::deg);
    RPos = -Right.position(rotationUnits::deg);
    SPos = Side.position(rotationUnits::deg);

    //Calculate distance traveled by tracking each wheel (INCHES)
    //Converts degrees to radians
    deltaDistL = ((LPos - LPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    deltaDistR = ((RPos - RPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    deltaDistS = ((SPos - SPrevPos) * M_PI / 180) * WHEEL_RADIUS;

    //Update previous values (DEGREES)
    LPrevPos = LPos;
    RPrevPos = RPos;
    SPrevPos = SPos;

    //Total change in each of the L and R encoders since last reset (INCHES)
    totalDeltaDistL += deltaDistL;
    totalDeltaDistR += deltaDistR;

    //Calculate the current absolute orientation (RADIANS)
    //currentAbsoluteOrientation = THETA_START - ( (totalDeltaDistL - totalDeltaDistR) / (LTrackRadius + RTrackRadius) );
    currentAbsoluteOrientation = (360 - Inertial.heading(rotationUnits::deg)) * M_PI / 180.0;

    //Calculate the change in the angle of the bot (RADIANS)
    deltaTheta = currentAbsoluteOrientation - previousTheta;

    //Update the previous Theta value (RADIANS)  
    previousTheta = currentAbsoluteOrientation;

    //If no turn, then translation
    if(deltaTheta == 0) {
      deltaXLocal = deltaDistS;
      // could be either L or R; if deltaTheta == 0, left = right
      deltaYLocal = deltaDistL;
    }
    //Else, calculate the new local position
    else {
      //Calculate the changes in the X and Y values (INCHES)
      //Distance = 2 * Radius * sin(deltaTheta / 2)
      deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistS / deltaTheta) + STrackRadius);
      deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistR / deltaTheta) - RTrackRadius);
    }

    //The average angle of the robot during its arc (RADIANS)
    avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

    deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
    deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));

    //Angle wrapping
    while(currentAbsoluteOrientation >= 2 * M_PI) {
      currentAbsoluteOrientation -= 2 * M_PI;
    }
    
    while(currentAbsoluteOrientation < 0) {
      currentAbsoluteOrientation += 2 * M_PI;
    }

    //Update global positions
    xPosGlobal += deltaXGlobal;
    yPosGlobal += deltaYGlobal;


    if (Controller1.ButtonB.PRESSED){
      std::cout << "x: " << xPosGlobal << std::endl;
      std::cout << "y: " << yPosGlobal << std::endl;
      std::cout << "heading: " << currentAbsoluteOrientation << std::endl << std::endl;
    }

    // Brain.Screen.setCursor(4,8);
    // Brain.Screen.print(deltaDistS/deltaTheta);
    // Brain.Screen.setCursor(5,8);
    // Brain.Screen.print((deltaDistL+deltaDistR)/2);
    // Brain.Screen.setCursor(6,8);
    // Brain.Screen.print(deltaDistS);
    // Brain.Screen.setCursor(7,8);
    // Brain.Screen.print(deltaTheta);


    //loop every 20 milliseconds
    task::sleep(10);

  }
  return 1;
}
