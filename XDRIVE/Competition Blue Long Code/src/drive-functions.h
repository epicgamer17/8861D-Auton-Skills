#include "vex.h"; 


class DriveFunctions {
  public: 
  float k = 5.29/360; //Turning constant 

  void move(double dist, double speed) { //In metres. 1 tile = 45.72 cm = 0.4572 m
    dist = dist/(2*M_PI*0.041275);
    backLeft.startSpinFor(dist, rev, speed-0, velocityUnits::pct); //maybe use startSpinFor instead
    frontLeft.startSpinFor(dist, rev, speed-0, velocityUnits::pct);
    backRight.startSpinFor(dist, rev, speed-0, velocityUnits::pct);
    frontRight.startSpinFor(dist, rev, speed-0, velocityUnits::pct);
    while (frontRight.isSpinning() && backLeft.isSpinning()) {
      wait(200, msec);
      if (abs(frontRight.velocity(rpm)) < 1 && abs(backLeft.velocity(rpm) < 1)) {
        backLeft.stop();
        frontLeft.stop();
        backRight.stop();
        frontRight.stop();
        break;
      } 
    }
  }

  void moveRight(double dist, double speed) { //In metres. 1 tile = 45.72 cm = 0.4572 m
    dist = dist/(2*M_PI*0.041275);
    backLeft.startSpinFor(-dist, rev, speed, velocityUnits::pct); //maybe use startSpinFor instead
    frontLeft.startSpinFor(dist, rev, speed, velocityUnits::pct);
    backRight.startSpinFor(dist, rev, speed, velocityUnits::pct);
    frontRight.startSpinFor(-dist, rev, speed, velocityUnits::pct);
    while (frontRight.isSpinning() && backLeft.isSpinning()) {
      wait(200, msec);
      if (abs(frontRight.velocity(rpm)) < 1 && abs(backLeft.velocity(rpm) < 1)) {
        backLeft.stop();
        frontLeft.stop();
        backRight.stop();
        frontRight.stop();
        break;
      } 
    }
  }

  //Turn Function 
  void turn(double a) {  
    //Will be adjusted for angle, counter clockwise: 
    backLeft.startSpinFor(-a*k, rev, 50, velocityUnits::pct);
    frontLeft.startSpinFor(-a*k, rev, 50, velocityUnits::pct);
    backRight.startSpinFor(a*k, rev, 50, velocityUnits::pct);
    frontRight.startSpinFor(a*k, rev, 50, velocityUnits::pct);
    while (frontRight.isSpinning() && backRight.isSpinning()) {
      wait(200, msec);
    }
  }

  void index() {
    indexer.startSpinFor(1, rev, 100, velocityUnits::pct);
  }
};

