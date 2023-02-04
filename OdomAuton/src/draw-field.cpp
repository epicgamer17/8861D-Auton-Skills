#include "draw-field.h"

float goalSize = 0.254 * 66.67;
float discSize = 0.1016 * 66.67;

float robotSize = 22.5; //0.381 * 66.67 * 0.5;

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
    //Home Row Lines
    // Brain.Screen.drawLine(x + 40, y, x + 40, y + 240);
    // Brain.Screen.drawLine(x, y + 118, x + 40, y + 118);
    // Brain.Screen.drawLine(x, y + 122, x + 40, y + 122);

    // Brain.Screen.drawLine(x + 200, y, x + 200, y + 240);
    // Brain.Screen.drawLine(x + 200, y + 118, x + 240, y + 118);
    // Brain.Screen.drawLine(x + 200, y + 122, x + 240, y + 122);
    
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

    robotX = globalX * 66.67;
    robotY = -globalY * 66.67;


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


    // Brain.Screen.setPenColor(green);

    // //Calculate offsets for box around robot
    // lineOffset1 = sqrt(2) * robotSize * cos(IMUCurrentAbsoluteOrientation + M_PI_4);
    // lineOffset2 = sqrt(2) * robotSize * cos(IMUCurrentAbsoluteOrientation - M_PI_4);

    // robotX = IMUGlobalX * 66.67;
    // robotY = -IMUGlobalY * 66.67;


    // //Draw the Robot
    // Brain.Screen.setPenColor(black);
    // Brain.Screen.drawLine(robotX + lineOffset1, 240 + robotY - lineOffset2, robotX + lineOffset2, 240 + robotY + lineOffset1);
    // Brain.Screen.drawLine(robotX + lineOffset2, 240 + robotY + lineOffset1, robotX - lineOffset1, 240 + robotY + lineOffset2);
    // Brain.Screen.drawLine(robotX - lineOffset1, 240 + robotY + lineOffset2, robotX - lineOffset2, 240 + robotY - lineOffset1);
    // Brain.Screen.drawLine(robotX - lineOffset2, 240 + robotY - lineOffset1, robotX + lineOffset1, 240 + robotY - lineOffset2);

    // //calculate where to place forward line
    // headingX = 10 * cos(IMUCurrentAbsoluteOrientation);
    // headingY = 10 * sin(IMUCurrentAbsoluteOrientation);

    // //Draw Heading Line
    // Brain.Screen.drawLine(robotX, 240 + robotY, robotX + headingX, 240 + robotY - headingY);  


    //DESIRED STUFF
    lineOffsetDesired1 = sqrt(2) * robotSize * cos(desiredHeading + M_PI_4);
    lineOffsetDesired2 = sqrt(2) * robotSize * cos(desiredHeading - M_PI_4);
    
    //Draw Desired Location
    drawDisc(desiredX * 66.67, 240 + (-desiredY * 66.67));
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.drawLine(desiredX + lineOffsetDesired1, 240 + desiredY - lineOffsetDesired2, desiredX + lineOffsetDesired2, 240 + desiredY + lineOffsetDesired1);
    Brain.Screen.drawLine(desiredX + lineOffsetDesired2, 240 + desiredY + lineOffsetDesired1, desiredX - lineOffsetDesired1, 240 + desiredY + lineOffsetDesired2);
    Brain.Screen.drawLine(desiredX - lineOffsetDesired1, 240 + desiredY + lineOffsetDesired2, desiredX - lineOffsetDesired2, 240 + desiredY - lineOffsetDesired1);
    Brain.Screen.drawLine(desiredX - lineOffsetDesired2, 240 + desiredY - lineOffsetDesired1, desiredX + lineOffsetDesired1, 240 + desiredY - lineOffsetDesired2);

    //calculate where to place forward line
    desiredHeadingX = 10 * cos(desiredHeading);
    desiredHeadingY = 10 * sin(desiredHeading);
    
    Brain.Screen.setPenColor(yellow);

    //Draw Heading Line
    Brain.Screen.drawLine(desiredX, 240 + desiredY, desiredX + desiredHeadingX, 240 + desiredY - desiredHeadingY);

    Brain.Screen.setPenColor(black);

    //Brain.Screen.setCursor(3, 7);
    //Brain.Screen.print(yPosGlobal);
    //Uncomment To Animate the Robot
    //robotX += 1;
    //currentAbsoluteOrientation += 0.1;

    Brain.Screen.setCursor(1, 25);
    Brain.Screen.print("Orientation: %f", currentAbsoluteOrientation * 180 / M_PI);

    Brain.Screen.setCursor(2, 25);
    Brain.Screen.print("X: %f", globalX);

    Brain.Screen.setCursor(3, 25);
    Brain.Screen.print("Y: %f", globalY);

    Brain.Screen.setCursor(7, 25);
    Brain.Screen.print("Joy Stick Axis 1: %f", Controller1.Axis1.value());

    Brain.Screen.setCursor(8, 25);
    Brain.Screen.print("Joy Stick Axis 3: %f", Controller1.Axis3.value());

    Brain.Screen.setCursor(9, 25);
    Brain.Screen.print("Joy Stick Axis 4: %f", Controller1.Axis4.value());

    Brain.Screen.setCursor(10, 25);
    Brain.Screen.print("Fly Wheel 1 Speed: %f", flyWheel1.velocity(rpm));
    Brain.Screen.setCursor(11, 25);
    Brain.Screen.print("Fly Wheel 2 Speed: %f", flyWheel2.velocity(rpm));
    Brain.Screen.setCursor(12, 25);
    Brain.Screen.print("Flywheel Voltage: %f", flyWheelSpeed);


    task::sleep(20);
  }

  return 1;
}