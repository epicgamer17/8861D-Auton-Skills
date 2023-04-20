#include "vex.h"
#include "drive.h"

using namespace vex;

void Drive::SetVoltage(double left, double right){
  frontLeft.spin(fwd,left,volt);
  midLeft.spin(fwd,left,volt);
  backLeft.spin(fwd,left,volt);
  frontRight.spin(fwd,right,volt);
  midRight.spin(fwd,right,volt);
  backRight.spin(fwd,right,volt);
}

float leftSpeed = 0;
  float rightSpeed = 0;

void Drive::RobotOriented(){
  
  double Turn = 0;
  double Power = 0;

  if (abs(Controller1.Axis1.value())>10){
    Turn = -Controller1.Axis1.value()/15;
  }
  else {
   Turn = 0;
  }
  if (abs(Controller1.Axis3.value())>10){
    Power = Controller1.Axis3.value()/11;
  }
  else {
   Power = 0;
  }
  
  double Left = Turn - Power;
  double Right = Turn + Power;

  frontRight.spin(fwd, Right, voltageUnits::volt);
  midRight.spin(fwd, Right, voltageUnits::volt);
  backRight.spin(fwd, Right, voltageUnits::volt);
    
  frontLeft.spin(reverse, Left, voltageUnits::volt);
  midLeft.spin(reverse, Left, voltageUnits::volt);
  backLeft.spin(reverse, Left, voltageUnits::volt);



}

void Drive::expansion(){
  Brain.resetTimer();
  while (1){
    if ((Brain.timer(seconds)>1) & (Controller2.ButtonUp.pressing() || (Controller1.ButtonUp.pressing()))){
   string.set(true);
   break;
  }
  else {
    string.set(false);
  }
  this_thread::sleep_for(30);
  }
}
int toggle = 0;
void Drive::Intake(){
  if (Controller1.ButtonL2.pressing() && (flyWheel.velocity(rpm) > 0)) {
    intake.spin(fwd,12,volt);
  }
  else if (Controller1.ButtonR2.pressing()){
    intake.spin(reverse,12,volt);
  }
  else {
    intake.stop();
  }
}

extern bool highvoltage = false;
bool togglefly = false;
void Drive::Shoot(){

    if(!(togglefly)){
      togglefly = true;
    }
    else if (togglefly){
      togglefly = false;
    }
    

  if(togglefly && highvoltage){
    flyWheel.spin(fwd,12,volt);
  }
  else if (togglefly && !(highvoltage)){
    flyWheel.spin(fwd,10.75,volt);
  }
  else {
    flyWheel.stop();
  }

}

void Drive::Move(int x, int spd){
  frontLeft.startRotateFor(directionType::fwd,x,rotationUnits::deg,spd,velocityUnits::pct);
  midLeft.startRotateFor(directionType::fwd,x,rotationUnits::deg,spd,velocityUnits::pct);
  backLeft.startRotateFor(directionType::fwd,x,rotationUnits::deg,spd,velocityUnits::pct);
  frontRight.startRotateFor(directionType::rev,x,rotationUnits::deg,spd,velocityUnits::pct);
  midRight.startRotateFor(directionType::rev,x,rotationUnits::deg,spd,velocityUnits::pct);
  backRight.rotateFor(directionType::rev,x,rotationUnits::deg,spd,velocityUnits::pct);
}

void Drive::Turn(int x){
  frontLeft.startRotateFor(fwd,x,degrees);
  midLeft.startRotateFor(fwd,x,degrees);
  backLeft.startRotateFor(fwd,x,degrees);
  frontRight.startRotateFor(fwd,x,degrees);
  midRight.startRotateFor(fwd,x,degrees);
  backRight.rotateFor(fwd,x,degrees);
}

void Drive::Inroll(int x){
  intake.startRotateFor(forward, x, degrees);
}


void Drive::Shots(int spd, int deg, int indexdeg){
  intake.resetRotation();
  while(1){
   if ((intake.rotation(rotationUnits::deg)) >= deg){
    flyWheel.stop();
    intake.stop();
    break;
  }
  else{
    flyWheel.spin(fwd,12,volt);
    if(flyWheel.velocity(rpm)>spd){
      intake.spinFor(directionType::fwd, indexdeg, rotationUnits::deg, 600,velocityUnits::rpm);
    }
  } 
  wait(5,msec);
  }
 
}
extern bool flappytog = false;

void Drive::flappy(){
  //if (Controller1.ButtonX.pressing()){
    if(flappytog == false){
      angleAdjuster.set(true);
      flappytog= true;
    
    }
    else if (flappytog){
      angleAdjuster.set(false);
      flappytog =false;
    }
  //}
  
}

extern float FlySpd = 10.75;

int Drive::updateController2(){
  Controller2.Screen.clearScreen();
  while (1){

    if (Controller2.ButtonX.pressing()){
      FlySpd = 12;
      

    }
    else if (Controller2.ButtonA.pressing()){
      FlySpd = 10.75;
      
    }

      Controller2.Screen.setCursor(3,1);
      Controller2.Screen.print("Flywheel voltage:: %f", FlySpd);
  
    if(togglefly){
      flyWheel.spin(fwd,FlySpd,volt);
    }
    else{
      flyWheel.stop();
    }

    if (flappytog){
      Controller2.Screen.setCursor(1,1);
      Controller2.Screen.print("Flappy Status::  ON");
  }
  else if ( flappytog == false) {
      Controller2.Screen.setCursor(1,1);
      Controller2.Screen.print("Flappy Status:: OFF");
  }
  this_thread::sleep_for(100);
    
  }
  return 1;
}