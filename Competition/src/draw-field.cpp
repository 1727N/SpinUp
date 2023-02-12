#include "draw-field.h"

#define scale 240/144

double goalSize = 7.865 * scale;

int robotSize = 16.5 * scale / 2;

double lineOffset1 = 0;
double lineOffset2 = 0;

double headingX = 0;
double headingY = 0;

double robotX = 0;
double robotY = 0;

// FIELD IS 140 INCHES, WHILE FIELD DRAWING IS 240 UNITS

void drawRedGoal(int x, int y) {
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(red);
    Brain.Screen.drawCircle(x - goalSize/2, y - goalSize, goalSize);
    Brain.Screen.setFillColor("#666666");
    Brain.Screen.drawCircle(x - goalSize/2, y - goalSize, (double) 6.48 * scale);
}

void drawBlueGoal(int x, int y) {
    Brain.Screen.setFillColor(blue);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawCircle(x - goalSize, y - goalSize/2, goalSize);
    Brain.Screen.setFillColor("#666666");
    Brain.Screen.drawCircle(x - goalSize, y - goalSize/2, (double) 6.48 * scale);
}

int drawField () {
  int x = 0;
  int y = 0;
  while(1) {
    Brain.Screen.setFillColor("#666666");
    Brain.Screen.drawRectangle(x, y, 240, 240);

    Brain.Screen.setPenColor("#404040");

    //Horizontal lines
    for(int i = y + 40; i < y + 240; i += 40) {
      Brain.Screen.drawLine(x, i, x + 240, i);
    }

    //Vertical lines
    for(int i = x + 40; i < x + 240; i += 40) {
      Brain.Screen.drawLine(i, y, i, y + 240);
    }

    //Field Lines
    Brain.Screen.setPenColor("#dbdbdb");
    Brain.Screen.drawLine(x, y, x + 240, y + 240);

    //Alliance Lines
    
    
  //Goals
    
    drawBlueGoal(x + 27.73 * scale, y + 122.22 * scale);

    drawRedGoal(x + 122.22 * scale, y + 27.73 * scale);

    //Calculate offsets for box around robot
    lineOffset1 = sqrt(2) * robotSize * cos(currentAbsoluteOrientation + M_PI_4);
    lineOffset2 = sqrt(2) * robotSize * cos(currentAbsoluteOrientation - M_PI_4);

    robotX = xPosGlobal * scale;
    robotY = -yPosGlobal * scale;


    //Draw the Robot
    Brain.Screen.setPenColor(blue);
    Brain.Screen.setPenWidth(5);
    Brain.Screen.drawLine(robotX + lineOffset1, 240 + robotY - lineOffset2, robotX + lineOffset2, 240 + robotY + lineOffset1);
    Brain.Screen.drawLine(robotX + lineOffset2, 240 + robotY + lineOffset1, robotX - lineOffset1, 240 + robotY + lineOffset2);
    Brain.Screen.drawLine(robotX - lineOffset1, 240 + robotY + lineOffset2, robotX - lineOffset2, 240 + robotY - lineOffset1);
    Brain.Screen.drawLine(robotX - lineOffset2, 240 + robotY - lineOffset1, robotX + lineOffset1, 240 + robotY - lineOffset2);
    Brain.Screen.setPenColor(black);

    //calculate where to place forward line
    headingX = 10 * cos(currentAbsoluteOrientation);
    headingY = 10 * sin(currentAbsoluteOrientation);

    //Draw Heading Line
    Brain.Screen.drawLine(robotX, 240 + robotY, robotX + headingX, 240 + robotY - headingY);

    // Brain.Screen.setCursor(3, 7);
    // Brain.Screen.print(yPosGlobal);
    //Uncomment To Animate the Robot
    // robotX += 1;
    // currentAbsoluteOrientation += 0.1;
    Brain.Screen.setFillColor(green);

    Brain.Screen.setCursor(2, 30);
    Brain.Screen.print("Orientation: %f", currentAbsoluteOrientation * 180 / M_PI);

    Brain.Screen.setCursor(4, 30);
    Brain.Screen.print("X: %f", xPosGlobal);

    Brain.Screen.setCursor(6, 30);
    Brain.Screen.print("Y: %f", yPosGlobal);

    task::sleep(20);
  }

  return 1;
}