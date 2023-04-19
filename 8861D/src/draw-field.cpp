#include "draw-field.h"

float goalSize = 10;
float discSize = 4;

float robotSize = 15;

double lineOffset1 = 0;
double lineOffset2 = 0;

double lineOffsetDesired1 = 0;
double lineOffsetDesired2 = 0;


double headingX = 0;
double headingY = 0;

double desiredHeadingX = 0;
double desiredHeadingY = 0;


double robotX = 0;
double robotY = 0;

void drawGoal(int x, int y, color col) {
    Brain.Screen.setFillColor(col);
    Brain.Screen.setPenColor(col);
    Brain.Screen.drawCircle(x, y, goalSize);
    Brain.Screen.setFillColor("#666666");
    Brain.Screen.drawCircle(x, y, goalSize - 0.1);
}

void drawDisc(int x, int y) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawCircle(x, y, discSize);
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawCircle(x, y, discSize - 0.1);
}


int drawField () {
  int x = 0;
  int y = 0;
  while(1) {
    Brain.Screen.setFillColor("#666666");
    Brain.Screen.drawRectangle(x, y, 240, 240);

    Brain.Screen.setPenColor("#404040");
    //horizontal lines
    for(int i = y + 40; i < y + 240; i+=40) {
      Brain.Screen.drawLine(x, i, x + 240, i);
    }

    //vertical lines
    for(int i = x + 40; i < x + 240; i+=40) {
      Brain.Screen.drawLine(i, y, i, y + 240);
    }

    //Field Lines
    Brain.Screen.setPenColor("#dbdbdb");
    //Match Auton line
    Brain.Screen.drawLine(10, 240, 240, 10);
    Brain.Screen.drawLine(0, 230, 230, 0);
    Brain.Screen.drawLine(0, 230, 230, 0);

    //Start Lines
    Brain.Screen.drawLine(240, 80, 220, 80);
    Brain.Screen.drawLine(200, 0, 200, 20);
    
    Brain.Screen.drawLine(0, 160, 20, 160);
    Brain.Screen.drawLine(40, 240, 40, 220);


    //Rollers
    Brain.Screen.setPenColor(blue);

    Brain.Screen.drawRectangle(0, 180, 5, 20, blue);
    Brain.Screen.drawRectangle(40, 235, 20, 5, blue);

    Brain.Screen.drawRectangle(235, 40, 5, 20, blue);
    Brain.Screen.drawRectangle(180, 0, 20, 5, blue);

    //Low Goals 
    Brain.Screen.setPenColor(blue);
    Brain.Screen.drawLine(40, 80, 80, 80);
    Brain.Screen.drawLine(80, 80, 80, 40);

    Brain.Screen.setPenColor(red);
    Brain.Screen.drawLine(160, 160, 200, 160);
    Brain.Screen.drawLine(160, 160, 160, 200);

  //Goals
    int tileLength = (40 / 23); 
    //Top Left
    drawGoal(x + 27.83, y + 27.83, red);
    //Bottom Right
    drawGoal(x + 240 -  27.83, y + 240 -  27.83, blue);

    // drawDisc(x + 240 -  27.83, y + 240 -  27.83);

    //Calculate offsets for box around robot
    lineOffset1 = sqrt(2) * robotSize * cos(currentAbsoluteOrientation + M_PI_4);
    lineOffset2 = sqrt(2) * robotSize * cos(currentAbsoluteOrientation - M_PI_4);

    robotX = globalX * 1.67;
    robotY = -globalY * 1.67;


    //Draw the Robot
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawLine(robotX + lineOffset1, 240 + robotY - lineOffset2, robotX + lineOffset2, 240 + robotY + lineOffset1);
    Brain.Screen.drawLine(robotX + lineOffset2, 240 + robotY + lineOffset1, robotX - lineOffset1, 240 + robotY + lineOffset2);
    Brain.Screen.drawLine(robotX - lineOffset1, 240 + robotY + lineOffset2, robotX - lineOffset2, 240 + robotY - lineOffset1);
    Brain.Screen.drawLine(robotX - lineOffset2, 240 + robotY - lineOffset1, robotX + lineOffset1, 240 + robotY - lineOffset2);

    //calculate where to place forward line
    headingX = 10 * cos(currentAbsoluteOrientation);
    headingY = 10 * sin(currentAbsoluteOrientation);

    //Draw Heading Line
    Brain.Screen.drawLine(robotX, 240 + robotY, robotX + headingX, 240 + robotY - headingY);  

    //DESIRED STUFF
    lineOffsetDesired1 = sqrt(2) * robotSize * cos(desiredHeading + M_PI_4);
    lineOffsetDesired2 = sqrt(2) * robotSize * cos(desiredHeading - M_PI_4);
    
    // // //Draw Desired Location
    // // drawDisc(desiredX, 240 + (-desiredY));
    // // Brain.Screen.setPenColor(yellow);
    // // Brain.Screen.drawLine(desiredX + lineOffsetDesired1, 240 + desiredY - lineOffsetDesired2, desiredX + lineOffsetDesired2, 240 + desiredY + lineOffsetDesired1);
    // // Brain.Screen.drawLine(desiredX + lineOffsetDesired2, 240 + desiredY + lineOffsetDesired1, desiredX - lineOffsetDesired1, 240 + desiredY + lineOffsetDesired2);
    // // Brain.Screen.drawLine(desiredX - lineOffsetDesired1, 240 + desiredY + lineOffsetDesired2, desiredX - lineOffsetDesired2, 240 + desiredY - lineOffsetDesired1);
    // // Brain.Screen.drawLine(desiredX - lineOffsetDesired2, 240 + desiredY - lineOffsetDesired1, desiredX + lineOffsetDesired1, 240 + desiredY - lineOffsetDesired2);

    // // //calculate where to place forward line
    // // desiredHeadingX = 10 * cos(desiredHeading);
    // // desiredHeadingY = 10 * sin(desiredHeading);
    
    // Brain.Screen.setPenColor(yellow);

    // //Draw Heading Line
    // Brain.Screen.drawLine(desiredX, 240 + desiredY, desiredX + desiredHeadingX, 240 + desiredY - desiredHeadingY);


    // Brain.Screen.setPenColor(opticalSensor.hue());
    // Brain.Screen.setFillColor(opticalSensor.color());
    
    // Brain.Screen.drawCircle(120, 120, 20);


    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);

    Brain.Screen.setCursor(1, 25);
    Brain.Screen.print("Orientation: %f", currentAbsoluteOrientation * 180 / M_PI);

    Brain.Screen.setCursor(2, 25);
    Brain.Screen.print("X: %f", globalX);

    Brain.Screen.setCursor(3, 25);
    Brain.Screen.print("Y: %f", globalY);

    Brain.Screen.setCursor(4, 25);
    Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel.velocity(rpm));
    Brain.Screen.setCursor(5, 25);
    Brain.Screen.print("Flywheel Voltage: %f", flyWheelSpeed);
    Brain.Screen.setCursor(7, 25);
    Brain.Screen.print("Sdw: %f", sidewaysRotation.position(degrees));
    Brain.Screen.setCursor(8, 25);

    Brain.Screen.print("Fwd: %f", forwardRotation.position(degrees) * (2.75*M_PI)/360);
    Brain.Screen.setCursor(9, 25);
    Brain.Screen.print("Desired Fwd: %f", desiredForwardValue);
    Brain.Screen.setCursor(10, 25);
    Brain.Screen.print("Drive Powerd: %f", drivePowerPID);

    Brain.Screen.setCursor(11, 25);
    Brain.Screen.print("Desired Heading: %f", desiredHeading);
    Brain.Screen.setCursor(12, 25);
    Brain.Screen.print("Turn Error: %f", turnError);

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Fly Wheel Speed: %.3f", flyWheelSpeed);
    Controller1.Screen.newLine();


    task::sleep(20);
  }

  return 1;
}