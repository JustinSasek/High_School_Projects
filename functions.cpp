#include "vex.h"
using namespace vex;

//Global Variables
  //Automatic
  bool updateLift = false;  //Moves lift to currentLiftPosition
  bool updateLiftLevels = false;  //Changes currentLiftPosition with controller buttons
  bool updateMogoIntakeSensor = false;  //Intakes goal when goal is detected
  bool updateMogoIntakeColorSensor = false;  //Determines goal color to activate rings

  bool updateClampSensor = false;  //Closes clamp when robot hits goal
  bool updateMiddleWheel = false;

  //Drive Train
  bool trackingWheels = false;  //Uses tracking wheel instead of internal rotary encoders positioning
  bool middleWheelOn = false;  //Activates middle wheel when driving forward
  int lastStopPosition = 0;  //Records middle position when robot stops
  
  //Mogo Intake Status
  int lastMogoIntakeTime = 0;  //Records last time mogoIntake was on

  //Lift Status
  int currentLiftLevel = 0;  //Moves lift to this postition
  int liftError = 0;  //How far off the lift is from its goal position
  double liftSpeed = 0;  //How fast the goal is currently moving
  bool firstTimeLifting = true;  //If moving for the first time, it will lift higher

  //Ring Intake Status
  bool intakingRings = false;  //ring intake is on

  //Clamp Status
  bool clamped = false;  //clamp is on
  int lastClampedTime = 0;  //last time clamp was on
  
  //Mid Move Status
  bool completed[6]  = {1, 1, 1, 1, 1, 1};  //Records if mid moves have been completed
  int midDistance[6] = {0, 0, 0, 0, 0, 0};  //When the mid moves should happen
  int midHeight = 0;  //Records lift level for lift mid move 

  //Odom
  double currentMoveDistance = 0;  //Current distance of the current move

  //target coordinates
  double targetX = 0;  //in  
  double targetY = 0;  //in
  double targetRot = 0;  //rad off +x axis //0 is forward

  //current coordinates
  double currentX = 0; 
  double currentY = 0;
  double currentRot = 0;
  double currentLeft = 0;  //Tracking wheel postitions
  double currentRight = 0;

  double PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706;

  //UI
  int settings = 4;  //Boxes in UI
  int currentScreen = 1;
  //Screen    Description
  //1         Main
  //2         Edit
  //3         Change "auton"
  //4         Printouts
  int border = 20;  //pixels
  int width = 480;
  int height = 240;

//Math
  double radiusFromXY(double x, double y) {
    return sqrt(x*x + y*y);
  }
  double degFromXY(double x, double y) {
    double deg = 0;
    if(x != 0) {
      deg = atan(y/x);
    }
    else {
      deg += PI/2;
    }
    if(x < 0) {
      deg += PI;
    }
    return deg;
  }
  double constrain(double deg) {
    while(deg < 0) {
      deg += 2*PI;
    }
    while(deg > 2*PI) {
      deg -= 2*PI;
    }
    return deg;
  }
  double taylorCos(double x) {
    return 1-0.5*x*x;
  }  //for small angles
  double taylorSin(double x) {
    return x-x*x*x/6;
  }  //for small angles
  double absValue(double x) {
    if(x<0) {
      x*=-1;
    }
    return x;
  }
//Auton:  Actions
  void aRaiseLift(int height) {
    liftLevel[2] = height;
    liftLevel[3] = height;
    if(currentLiftLevel<=1) {
      currentLiftLevel += 2;
    }
  } 
  void aRaiseLift(void) {
    aRaiseLift(315);
  }   
  void aLowerLift(void) {
    if(currentLiftLevel>=2) {
      currentLiftLevel -= 2;
    }
  }
  void aCloseClamp(void) {
    if(!clamped) {
      clamped = true;
      currentLiftLevel += 1;
    }
  }
  void aOpenClamp(void) {
    if(clamped) {
      clamped = false;
      currentLiftLevel -= 1;
    }
  }
  void aMogoIntake(void) {
    mogoIntake.set(true);
  }
  void aMogoOuttake(void) {
    mogoIntake.set(false);
  }
//        Updates
  void aResetPosition(void) {
    currentX = 0;
    currentY = 0;
    currentRot = 0;
    currentLeft = (double)(lfDrive.position(deg) + lbDrive.position(deg)) / 4 / degreesperinch;
    currentRight = (double)(rfDrive.position(deg) + rbDrive.position(deg)) / 4 / degreesperinch;
  }  //resets coordinates to 0
  void aUpdatePosition(void) {
    double left = (double)(lfDrive.position(deg) + lbDrive.position(deg)) / 4 / degreesperinch;
    double right = (double)(rfDrive.position(deg) + rbDrive.position(deg)) / 4 / degreesperinch;
    
    double SR = right - currentRight;
    double SL = left - currentLeft;
    double r;
    double theta2;
    if(SR==SL) {
      r = SR / degreesperinch;
    }
    else {
      r = (SR + SL) / 2 / degreesperinch;
      theta2 = (SR - SL) * PI / 360 / degreesperdegree;
      currentRot = constrain(theta2 + currentRot);
    }
    currentX += r * cos(currentRot + PI/2);
    currentY += r * sin(currentRot + PI/2);
    currentMoveDistance += r;
    currentLeft = left;
    currentRight = right;
  }  //updates coordinates
  void aUpdateMidMoves(void) {
    if(!completed[0] && currentMoveDistance > midDistance[0]) {
      completed[0] = true;
      aRaiseLift(midHeight);
    }
    if(!completed[1] && currentMoveDistance > midDistance[1]) {
      completed[1] = true;
      aLowerLift();
    }
    if(!completed[2] && currentMoveDistance > midDistance[2]) {
      completed[2] = true;
      aMogoIntake();
    }
    if(!completed[3] && currentMoveDistance > midDistance[3]) {
      completed[3] = true;
      aMogoOuttake();
    }
    if(!completed[4] && currentMoveDistance > midDistance[4]) {
      completed[4] = true;
      aCloseClamp();
    }
    if(!completed[5] && currentMoveDistance > midDistance[5]) {
      completed[5] = true;
      aOpenClamp();
    }
  }  //allows multitasking during moves
  void aUpdateLift(void) {
    //Target Variable
    if(!updateLiftLevels) {
      currentLiftLevel = 0;
    }
    liftError = liftLevel[currentLiftLevel] - (rLift.position(degrees) + lLift.position(degrees))/2;
    if(firstTimeLifting) {
      liftError += initialLiftPosition;
    }
    double liftTargetSpeed = aLiftMinSpeed + (aLiftMaxSpeed - aLiftMinSpeed) * liftError / aLiftDecel;
    if(liftError < 0) {
      liftTargetSpeed -= 2 * aLiftMinSpeed;
    }
    //Acceleration
    double liftAcceleration = liftTargetSpeed - liftSpeed;
    if (liftError > 0 && liftAcceleration > 2 * aLiftMaxSpeed / aLiftCycles) {
      liftAcceleration = (2 * aLiftMaxSpeed / aLiftCycles);
    }
    if (liftError < 0 && liftAcceleration < -2 * aLiftMaxSpeed / aLiftCycles) {
      liftAcceleration = (-2 * aLiftMaxSpeed / aLiftCycles);
    }
    liftSpeed += liftAcceleration;
    //Offset
    //Implement
    
    if(abs((int)liftError) > liftTolerance) {
      rLift.spin(forward, liftSpeed + aLiftOffsetSpeed, percent);
      lLift.spin(forward, liftSpeed + aLiftOffsetSpeed, percent);
    }
    else {
      rLift.stop(brakeType::hold);
      lLift.stop(brakeType::hold);
      liftSpeed = 0;
      firstTimeLifting = false;
    }
  }  //Input: currentLiftLevel
  void aUpdateClamp(void) {
    if(clamped) {
      lastClampedTime = Brain.Timer.time();
      clamp.set(false);
    }
    else if(!updateLift || Brain.Timer.time() - lastClampedTime > releaseDelay) {
      clamp.set(true);
    }
  }  //Input: clamped
  void aUpdateRingIntake(void) {
    if(Controller.ButtonY.pressing()) {
      ringIntake.spin(forward, -1 * aRingIntakeMaxSpeed, percent);
    }
    else if(intakingRings) {
      ringIntake.spin(forward, aRingIntakeMaxSpeed, percent);
    }
    else {
      ringIntake.stop(brakeType::coast);
    }
  }  //Input: intakingRings
  void aUpdateMogoIntakeSensor(void) {
    if(mogoIntakeSensor.isNearObject()) {
      aMogoIntake();
      if(Red) {
        if(mogoIntakeSensor.hue() < redColorThreshold) {
          intakingRings = true;
        }
      }
      else {
        if(mogoIntakeSensor.hue() > blueColorThreshold) {
          intakingRings = true;
        }
      }
    }
  }
  void aUpdateMiddleWheel(void) {
    if(middleWheelOn && driveSpeed > 0) {
      middleDrive.spin(forward, driveSpeed, percent);
      lastStopPosition = middleDrive.position(degrees);
    }
    else if(middleDrive.position(degrees) > lastStopPosition - middleWheelRetractionDistance) { 
      middleDrive.spin(reverse, middleWheelRetractionSpeed, percent);
    }
    else {
      middleDrive.stop(brakeType::hold);
    }
  } 
  void aUpdateAll(void) {
    aUpdatePosition();
    aUpdateMidMoves();
    if(updateLift) {
      aUpdateLift();
    }
    aUpdateClamp();
    aUpdateRingIntake();
    if(updateMogoIntakeSensor) {
      aUpdateMogoIntakeSensor();
    }
    if(updateMiddleWheel) {
      aUpdateMiddleWheel();
    }
  }
  void aWait(double sec) {
    int loopStartTime = Brain.Timer.time();
    while(Brain.Timer.time() - loopStartTime < sec * 1000) {
      aUpdateAll();
      wait(loopSleepTime, vex::timeUnits::msec);
    }
  }  //wait with updates
  void aTernminateMidMoves(void) {
    currentMoveDistance = 0;
    if(!completed[0]) {
      completed[0] = true;
      aRaiseLift(midHeight);
    }
    if(!completed[1]) {
      completed[1] = true;
      aLowerLift();
    }
    if(!completed[2]) {
      completed[2] = true;
      aMogoIntake();
    }
    if(!completed[3]) {
      completed[3] = true;
      aMogoOuttake();
    }
    if(!completed[4]) {
      completed[4] = true;
      aCloseClamp();
    }
    if(!completed[5]) {
      completed[5] = true;
      aOpenClamp();
    }
  }  //activates mid moves if not completed by end of move
//        Mid Move Actions
  //performs actions in the mdidle of moves
  void aMidRaiseLift(int height, double dist) {
    completed[0] = false;
    midHeight = height;
    midDistance[0] = dist;
  }
  void aMidLowerLift(double dist) {
    completed[1] = false;
    midDistance[1] = dist;
  }
  void aMidMogoIntake(double dist) {
    completed[2] = false;
    midDistance[2] = dist;
  }
  void aMidMogoOuttake(double dist) {
    completed[3] = false;
    midDistance[3] = dist;
  }
  void aMidCloseClamp(double dist) {
    completed[4] = false;
    midDistance[4] = dist;
  }
  void aMidOpenClamp(double dist) {
    completed[5] = false;
    midDistance[5] = dist;
  }
//Motion  Motion
  void goTo(int type, bool Forward, double sec) {
    double xError = targetX - currentX;
    double yError = targetY - currentY;
    double turnError = degFromXY(xError, yError) - currentRot - PI/2;
      if(!Forward) {
        turnError += PI;
      }
      turnError = constrain(turnError + PI) - PI;
    double driveError = radiusFromXY(xError, yError) * cos(turnError);

    int loopStartTime = Brain.Timer.time();
    int n =  lfDrive.current(percent) + lbDrive.current(percent) + rfDrive.current(percent) + rbDrive.current(percent);
    double driveSpeed = n/4;
    n =  lfDrive.current(percent) + lbDrive.current(percent) - rfDrive.current(percent) - rbDrive.current(percent);
    double turnSpeed = n/4;

    while((absValue(driveError) > 1 
          || (absValue(turnError) * 180 / PI > 1 && radiusFromXY(xError, yError) > lockCourseDistance))
          && Brain.Timer.time() - loopStartTime < 1000 * sec) {
      //Error
        xError = targetX - currentX;
        yError = targetY - currentY;
        turnError = degFromXY(xError, yError) - currentRot - PI/2;
          if(!Forward) {
            turnError += PI;
          }
          turnError = constrain(turnError + PI) - PI;

        driveError = radiusFromXY(xError, yError) * cos(turnError);
      //Target Variables
        double driveTargetSpeed;
        if(type == 3) {
          driveTargetSpeed = aDriveMaxSpeed;
        }
        else { 
          driveTargetSpeed = aDriveMinSpeed + (aDriveMaxSpeed - aDriveMinSpeed) * driveError / aDriveDecel;
          if(driveError < 0) {
            driveTargetSpeed -= 2 * aDriveMinSpeed;
          }
        }
        double turnTargetSpeed = 0;
        if(radiusFromXY(xError, yError) > lockCourseDistance) {
          turnTargetSpeed = -1 * aTurnMaxSpeed * turnError * 180 / PI / aTurnDecel / degreesperdegree;
        }

      //Acceleration
        double driveAcceleration = driveTargetSpeed - driveSpeed;
        if (driveError > 0 && driveAcceleration > 2 * aDriveMaxSpeed / aDriveCycles) {
          driveAcceleration = (2 * aDriveMaxSpeed / aDriveCycles);
        }
        if (driveError < 0 && driveAcceleration < -2 * aDriveMaxSpeed / aDriveCycles) {
          driveAcceleration = (-2 * aDriveMaxSpeed / aDriveCycles);
        }
        driveSpeed += driveAcceleration;

        double turnAcceleration = turnTargetSpeed - turnSpeed;
        if (turnError > 0 && turnAcceleration > 2 * aTurnMaxSpeed / aTurnCycles) {
          turnAcceleration = (2 * aTurnMaxSpeed / aTurnCycles);
        }
        if (turnError < 0 && turnAcceleration < -2 * aTurnMaxSpeed / aTurnCycles) {
          turnAcceleration = (-2 * aTurnMaxSpeed / aTurnCycles);
        }
        turnSpeed += turnAcceleration;
      //Constrain
        if(driveSpeed > aDriveMaxSpeed) {
          driveSpeed = aDriveMaxSpeed;
        }
        if(driveSpeed < -1 * aDriveMaxSpeed) {
          driveSpeed = -1*aDriveMaxSpeed;
        }
        double mid = driveSpeed;
        double rf = driveSpeed - turnSpeed;
        double rb = driveSpeed - turnSpeed;
        double lf = driveSpeed + turnSpeed;
        double lb = driveSpeed + turnSpeed;
        if(rf > 100) {
          rb *= 100/rf;
          lf *= 100/rf;
          lb *= 100/rf;
          mid *= 100/rf;
          rf = 100;
        }
        if(rf < -100) {
          rb *= -100/rf;
          lf *= -100/rf;
          lb *= -100/rf;
          mid *= -100/rf;
          rf = -100;
        }
        if(rb > 100) {
          rf *= 100/rb;
          lf *= 100/rb;
          lb *= 100/rb;
          mid *= 100/rb;
          rb = 100;
        }
        if(rb < -100) {
          rf *= -100/rb;
          lf *= -100/rb;
          lb *= -100/rb;
          mid *= -100/rb;
          rb = -100;
        }
        if(lf > 100) {
          rf *= 100/lf;
          rb *= 100/lf;
          lb *= 100/lf;
          mid *= 100/lf;
          lf = 100;
        }
        if(lf < -100) {
          rf *= -100/lf;
          rb *= -100/lf;
          lb *= -100/lf;
          mid *= -100/lf;
          lf = -100;
        }
        if(lb > 100) {
          rb *= 100/lb;
          rf *= 100/lb;
          lf *= 100/lb;
          mid *= 100/lb;
          lb = 100;
        }
        if(lb < -100) {
          rb *= -100/lb;
          rf *= -100/lb;
          lf *= -100/lb;
          mid *= -100/lb;
          lb = -100;
        }
      //Implement
      rfDrive.spin(forward, rf, percent);
      rbDrive.spin(forward, rb, percent);
      lfDrive.spin(forward, lf, percent);
      lbDrive.spin(forward, lb, percent);

      aUpdateAll();
      //wait(loopSleepTime, msec);
    }
    if(type == 0) {
      rfDrive.stop(brakeType::hold);
      rbDrive.stop(brakeType::hold);
      lfDrive.stop(brakeType::hold);
      lbDrive.stop(brakeType::hold); 
      aWait((double)aBrakeTime / 1000);
    }
    else if(type == 1) {
      rfDrive.stop(brakeType::coast);
      rbDrive.stop(brakeType::coast);
      lfDrive.stop(brakeType::coast);
      lbDrive.stop(brakeType::coast);  
    }
    aTernminateMidMoves();
  }
  void aForward(double inches, double sec, bool coast) {
        if(Reversed) {
      inches *= -1;
    }
    double startPosition;
    if(trackingWheels) {
      startPosition = (rightOdom.position(degrees) + 
      leftOdom.position(degrees))/2;
    }
    else {
      startPosition = (rfDrive.position(degrees) + 
      rbDrive.position(degrees) + 
      lfDrive.position(degrees) + 
      lbDrive.position(degrees))/4;
    }
    double error = inches * degreesperinch;
    int loopStartTime = Brain.Timer.time();
    double driveSpeed = 0;
    while(abs((int)error) > 3  && Brain.Timer.time() - loopStartTime < 1000 * sec) {
      //Target Variable
      if(trackingWheels) {
        error = inches * degreesperinch 
            - (rightOdom.position(degrees) + leftOdom.position(degrees))/2 
            + startPosition;
      }
      else {
        error = inches * degreesperinch - (rfDrive.position(degrees) + 
        rbDrive.position(degrees) + 
        lfDrive.position(degrees) + 
        lbDrive.position(degrees))/4 + startPosition;
      }
      
      double driveTargetSpeed = aDriveMinSpeed + (aDriveMaxSpeed - aDriveMinSpeed) * error / (aDriveDecel * degreesperinch);
      if(error < 0) {
        driveTargetSpeed -= 2 * aDriveMinSpeed;
      }
      //Acceleration
      double driveAcceleration = driveTargetSpeed - driveSpeed;
      if (error > 0 && driveAcceleration > 2 * aDriveMaxSpeed / aDriveCycles) {
        driveAcceleration = (2 * aDriveMaxSpeed / aDriveCycles);
      }
      if (error < 0 && driveAcceleration < -2 * aDriveMaxSpeed / aDriveCycles) {
        driveAcceleration = (-2 * aDriveMaxSpeed / aDriveCycles);
      }
      driveSpeed += driveAcceleration;
      //Constrain
      if(driveSpeed > aDriveMaxSpeed) driveSpeed = aDriveMaxSpeed;
      if(driveSpeed < -1*aDriveMaxSpeed) driveSpeed = -1*aDriveMaxSpeed;
      //Implement
      rfDrive.spin(forward, driveSpeed, percent);
      rbDrive.spin(forward, driveSpeed, percent);
      lfDrive.spin(forward, driveSpeed, percent);
      lbDrive.spin(forward, driveSpeed, percent);

      aUpdateAll();
      wait(loopSleepTime, msec);
    }
    if(coast) {
      rfDrive.stop(brakeType::coast);
      rbDrive.stop(brakeType::coast);
      lfDrive.stop(brakeType::coast);
      lbDrive.stop(brakeType::coast); 
    }
    else {
      rfDrive.stop(brakeType::hold);
      rbDrive.stop(brakeType::hold);
      lfDrive.stop(brakeType::hold);
      lbDrive.stop(brakeType::hold); 
      aWait((double)aBrakeTime / 1000);
    }
    aTernminateMidMoves();
  }  //in
  void aTurn(double rot, double sec, bool coast) {
    double startAngle;
    if(trackingWheels) {
      startAngle = (-1*rightOdom.position(degrees) +
      leftOdom.position(degrees))/2;
    }
    else {
      startAngle = (-1*rfDrive.position(degrees) +
      -1*rbDrive.position(degrees) + 
      lfDrive.position(degrees) + 
      lbDrive.position(degrees))/4;
    }
    double error = rot * degreesperdegree;
    int loopStartTime = Brain.Timer.time();
    double turnSpeed = 0;
    while(abs((int)error) > 3  && Brain.Timer.time() - loopStartTime < 1000 * sec) {
      //Target Variable
      if(trackingWheels) {
        error = rot * degreesperdegree 
            - (-1*rightOdom.position(degrees) + leftOdom.position(degrees))/2 
            + startAngle;
      }
      else {
        error = rot * degreesperdegree - (-1*rfDrive.position(degrees) + 
        -1*rbDrive.position(degrees) + 
        lfDrive.position(degrees) + 
        lbDrive.position(degrees))/4 + startAngle;
      }
      
      double turnTargetSpeed = aTurnMinSpeed + (aTurnMaxSpeed - aTurnMinSpeed) * error / (aTurnDecel * degreesperdegree);
      if(error < 0) {
        turnTargetSpeed -= 2 * aTurnMinSpeed;
      }
      //Acceleration
      double turnAcceleration = turnTargetSpeed - turnSpeed;
      if (error > 0 && turnAcceleration > 2 * aTurnMaxSpeed / aTurnCycles) {
        turnAcceleration = (2 * aTurnMaxSpeed / aTurnCycles);
      }
      if (error < 0 && turnAcceleration < -2 * aTurnMaxSpeed / aTurnCycles) {
        turnAcceleration = (-2 * aTurnMaxSpeed / aTurnCycles);
      }
      turnSpeed += turnAcceleration;
      //Constrain
      if(turnSpeed > aTurnMaxSpeed) turnSpeed = aTurnMaxSpeed;
      if(turnSpeed < -1*aTurnMaxSpeed) turnSpeed = -1*aTurnMaxSpeed;
      //Implement
      rfDrive.spin(forward, -1 * turnSpeed, percent);
      rbDrive.spin(forward, -1 * turnSpeed, percent);
      lfDrive.spin(forward, turnSpeed, percent);
      lbDrive.spin(forward, turnSpeed, percent);

      aUpdateAll();
      wait(loopSleepTime, msec);
    }
    if(coast) {
      rfDrive.stop(brakeType::coast);
      rbDrive.stop(brakeType::coast);
      lfDrive.stop(brakeType::coast);
      lbDrive.stop(brakeType::coast); 
    }
    else {
      rfDrive.stop(brakeType::hold);
      rbDrive.stop(brakeType::hold);
      lfDrive.stop(brakeType::hold);
      lbDrive.stop(brakeType::hold); 
      aWait((double)aBrakeTime / 1000);
    }
    aTernminateMidMoves();
  }  //robot degrees  //Clockwise when red
//        Motion Variants
  //Normal motion
  void aForward(double inches, double sec) {
    aForward(inches, sec, false);
  }
  void aForward(double inches) {
    aForward(inches, 10, false);
  }
  void aTurn(double rot, double sec) {  
    aTurn(rot, sec, false);    
  }
  void aTurn(double rot) {  
    aTurn(rot, 10, false);    
  }
  //Odometry motion
  void aGoForwardTo(double x, double y, double sec) {
    targetX = x;
    targetY = y;
    goTo(0, 1, sec);
  }  //moves and stops
  void aCoastForwardTo(double x, double y, double sec) {
    targetX = x;
    targetY = y;
    goTo(1, 1, sec);    
  }  //moves and coasts
  void aGlideForwardBy(double x, double y, double sec) {
    targetX = x;
    targetY = y;
    goTo(2, 1, sec);  
  }  //moves and doesn't stop
  void aGoBackwardTo(double x, double y, double sec) {
    targetX = x;
    targetY = y;
    goTo(0, 0, sec);
  }  //same but backward
  void aCoastBackwardTo(double x, double y, double sec) {
    targetX = x;
    targetY = y;
    goTo(1, 0, sec);    
  }
  void aGlideBackwardBy(double x, double y, double sec) {
    targetX = x;
    targetY = y;
    goTo(2, 0, sec);    
  }
  void aGoForwardTo(double x, double y) {
    targetX = x;
    targetY = y;
    goTo(0, 1, 10);
  }
  void aCoastForwardTo(double x, double y) {
    targetX = x;
    targetY = y;
    goTo(1, 1, 10);    
  }
  void aGlideForwardBy(double x, double y) {
    targetX = x;
    targetY = y;
    goTo(2, 1, 10);    
  }
  void aGoBackwardTo(double x, double y) {
    targetX = x;
    targetY = y;
    goTo(0, 0, 10);
  }
  void aCoastBackwardTo(double x, double y) {
    targetX = x;
    targetY = y;
    goTo(1, 0, 10);    
  }
  void aGlideBackwardBy(double x, double y) {
    targetX = x;
    targetY = y;
    goTo(2, 0, 10);    
  }
  //Uses odometry to turn
  void aTurnToHeading(double rot, double sec) {
    double deg = constrain(currentRot - rot*2*PI/360 + PI) - PI;
    aTurn(-1*deg, sec, 0);
  }
  void aCoastToHeading(double rot, double sec) {
    double deg = constrain(currentRot - rot*2*PI/360 + PI) - PI;
    aTurn(-1*deg, sec, 1);    
  }
  void aTurnToHeading(double rot) {
    double deg = constrain(currentRot - rot*2*PI/360 + PI) - PI;
    aTurn(-1*deg, 10, 0);
  }
  void aCoastToHeading(double rot) {
    double deg = constrain(currentRot - rot*2*PI/360 + PI) - PI;
    aTurn(-1*deg, 10, 1);    
  }
//Buttons
  //For user control
  void switchLift(void) {
    if(updateLiftLevels) {
      if(currentLiftLevel<=1) {
        currentLiftLevel += 2;
      }
      else {
        currentLiftLevel -= 2;
      }
    }
  }  //only works with updateLiftLevels
  void switchClamp(void) {
    clamped = !clamped;
    if(clamped) {
      currentLiftLevel += 1;
    }
    else {
      currentLiftLevel -= 1;
    }
  }
  void switchMogoIntake(void) {
    mogoIntake.set(!mogoIntake.value());
  }  //turns on automatic mode
  void reverseDriving(void) {
    Reversed = !Reversed;
  }
  void clampSensorPressed(void) {
    if(updateClampSensor && Brain.Timer.time() - lastClampedTime > clampSensorDelay) {
      aCloseClamp();
    }
  }
  void switchRingIntake(void) {
    intakingRings = !intakingRings;
  }
  void switchDriveAccleration(void) {
    driveAcceleration = !driveAcceleration;
  }
  void toggleMiddleWheel(void) {
    middleWheelOn = !middleWheelOn;
  }
  
//UI
  void UIInit(void) {
    Brain.Screen.clearScreen();
    if(Red) {
      Brain.Screen.setFillColor(color::red);
      Brain.Screen.setPenColor(color::red);
    }
    else {
      Brain.Screen.setFillColor(color::blue);
      Brain.Screen.setPenColor(color::blue);
    }
    Brain.Screen.drawRectangle(0, 0, width, height);
    Brain.Screen.setPenColor(color::white);
    Brain.Screen.setFont(fontType::prop60);
    Brain.Screen.printAt(160, 120, "7110A");
    Brain.Screen.setFont(fontType::prop30);
    int auton;
    if(Skills) {
      auton = skillsAuton;
    }
    else {
      auton = matchAuton;
    }
    Brain.Screen.printAt(110, 185, "Auton: %d", auton);
    if(Skills) {
      Brain.Screen.printAt(290, 185, "Skills");
    }
    else {
      Brain.Screen.printAt(290, 185, "Match");
    }
  }  //starts UI
  void updateUI(void) {
    int x = Brain.Screen.xPosition();
    int y = Brain.Screen.yPosition();
    if(currentScreen == 1) {
      if(x >= border && x <= width - border && y >= border && y <= height - border) {
        currentScreen = 2;
      }
    }
    else if(currentScreen == 2) {
      if(x >= border && x <= border + (width-6*border)/5 && y >= border && y <= border+height-2*border) {
        if(!demo && !programmingOveride) {
          currentScreen = 1;
        }          
      }
      int rows;
      if(settings < 3) {
        rows = 1;
      }
      else if(settings < 9) {
        rows = 2;
      } 
      else if(settings < 19) {
        rows = 3;
      } 
      else {
        rows = 4;
      } 
      int cols = (settings+rows-.00001)/rows;
      for(int i = 0; i < settings; i++) {
        int currentcol = i%cols;
        int currentrow = i/cols;
        int boxX = (width-6*border)/5 + border*(2+currentcol) + currentcol * (width-(cols+2)*border - (width-6*border)/5)/cols;
        int boxY = border*(1+currentrow) + currentrow * (height-(rows+1)*border)/rows;
        int w = (width-(cols+2)*border - (width-6*border)/5)/cols;
        int h = (height-(rows+1)*border)/rows;
        if(x >= boxX && x <= boxX + w && y >= boxY && y <= boxY + h) {
          if(i==0) {
            Red = !Red;
          }
          if(i==1) {
            if(!programmingOveride) {
              Skills = !Skills;
            }
          }
          if(i==2) {
            currentScreen = 3;
          }
          if(i==3) {
            if(demo) {
              demo = false;
              programmingOveride = true;
              Skills = true;
              liftLevel [2] = 315;
              liftLevel [3] = 315;
              releaseDelay = 300;
            }
            else if(programmingOveride) {
              programmingOveride = false;
            }
            else {
              demo = true;
            }
          }
        }
      }
    }
    else if(currentScreen == 3) {
      if(x >= border && x <= border + (width-6*border)/5 && y >= border && y <= border+height-2*border) {
        currentScreen = 2;
      }
      int rows;
      int totalAutons;
      if(Skills) {
        totalAutons = totalSkillsAutons;
      }
      else {
        totalAutons = totalMatchAutons;
      }
      if(totalAutons < 3) {
        rows = 1;
      }
      else if(totalAutons < 9) {
        rows = 2;
      } 
      else if(totalAutons < 19) {
        rows = 3;
      } 
      else {
        rows = 4;
      } 
      int cols = (totalAutons+rows-.00001)/rows;
      for(int i = 0; i < totalAutons; i++) {
        int currentcol = i%cols;
        int currentrow = i/cols;
        int boxX = (width-6*border)/5 + border*(2+currentcol) + currentcol * (width-(cols+2)*border - (width-6*border)/5)/cols;
        int boxY = border*(1+currentrow) + currentrow * (height-(rows+1)*border)/rows;
        int w = (width-(cols+2)*border - (width-6*border)/5)/cols;
        int h = (height-(rows+1)*border)/rows;
        if(x >= boxX && x <= boxX + w && y >= boxY && y <= boxY + h) {
          if(Skills) {
            skillsAuton = i+1;
          }
          else {
            matchAuton = i+1;
          }
          currentScreen = 2;
        }
      }
    }
    else if(currentScreen == 4) {
      currentScreen = 1;
    }
    Brain.Screen.clearScreen();
    if(currentScreen == 1) {
      if(Red) {
        Brain.Screen.setFillColor(color::red);
        Brain.Screen.setPenColor(color::red);
      }
      else {
        Brain.Screen.setFillColor(color::blue);
        Brain.Screen.setPenColor(color::blue);
      }
      Brain.Screen.drawRectangle(0, 0, width, height);
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.setFont(fontType::prop60);
      Brain.Screen.printAt(160, 120, "7110A");
      Brain.Screen.setFont(fontType::prop30);
      int auton;
      if(Skills) {
        auton = skillsAuton;
      }
      else {
        auton = matchAuton;
      }
      Brain.Screen.printAt(110, 185, "Auton: %d", auton);
      if(Skills) {
        Brain.Screen.printAt(290, 185, "Skills");
      }
      else {
        Brain.Screen.printAt(290, 185, "Match");
      }
    }
    if(currentScreen == 2) {
      if(demo || programmingOveride) {
        Brain.Screen.setFillColor(color(180, 0, 0));
      }
      else {
        Brain.Screen.setFillColor(color::black);
      }
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.drawRectangle(border, border, (width-6*border)/5, height-2*border);
      Brain.Screen.setFont(fontType::prop30);
      Brain.Screen.printAt(47, 70, "B");
      Brain.Screen.printAt(47, 110, "A");
      Brain.Screen.printAt(47, 150, "C");
      Brain.Screen.printAt(47, 190, "K");
      int rows;
      int font;
      if(settings < 3) {
        rows = 1;
        font = 60;
      }
      else if(settings < 9) {
        rows = 2;
        font = 30;
      } 
      else if(settings < 19) {
        rows = 3;
        font = 30;
      } 
      else {
        rows = 4;
        font = 20;
      } 
      int cols = (settings+rows-.00001)/rows;
      for(int i = 0; i < settings; i++) {
        int currentcol = i%cols;
        int currentrow = i/cols;
        int x = (width-6*border)/5 + border*(2+currentcol) + currentcol * (width-(cols+2)*border - (width-6*border)/5)/cols;
        int y = border*(1+currentrow) + currentrow * (height-(rows+1)*border)/rows;
        int w = (width-(cols+2)*border - (width-6*border)/5)/cols;
        int h = (height-(rows+1)*border)/rows;
        if(i==0) {
          if(Red) {
            Brain.Screen.setPenColor(color::red);
            Brain.Screen.setFillColor(color::red);
          }
          else {
            Brain.Screen.setPenColor(color::blue);
            Brain.Screen.setFillColor(color::blue);
          }
        }
        else if(i==1) {
          if(Skills) {
            Brain.Screen.setPenColor(color::white);
            Brain.Screen.setFillColor(color(70, 70, 70));
          }
          else {
            Brain.Screen.setPenColor(color::white);
            Brain.Screen.setFillColor(color::white);
          }
        }
        else if(i==2) {
          Brain.Screen.setPenColor(color(100, 100, 100));
          Brain.Screen.setFillColor(color(100, 100, 100));
        }
        else if(i==3) {
          Brain.Screen.setPenColor(color::black);
          Brain.Screen.setFillColor(color::black);
        }
        else {
          Brain.Screen.setPenColor(color(100, 100, 100));
          Brain.Screen.setFillColor(color(100, 100, 100));
        }
        
        Brain.Screen.drawRectangle(x, y, w, h);
        Brain.Screen.setPenColor(color::white);
        int xText = x+w/2;
        int yText = y+h/2;
        if(cols == 2) {
          xText += -23;
        }
        else if(cols == 3) {
          xText += -26;
        }
        if(rows == 2) {
          yText += 10;
        }
        if(font == 60) {
          Brain.Screen.setFont(fontType::prop60);
        }
        else if(font == 30) {
          Brain.Screen.setFont(fontType::prop30);
        }
        else {
          Brain.Screen.setFont(fontType::prop20);
        }
        if(i==0) {
          if(Red) {
            Brain.Screen.printAt(xText, yText, "Red");
          }
          else {
            Brain.Screen.printAt(xText-2, yText, "Blue");
          }
        }
        else if(i==1) {
          if(Skills) {
            Brain.Screen.printAt(xText-5, yText, "Skills");
          }
          else {
            Brain.Screen.setPenColor(color::black);
            Brain.Screen.printAt(xText-13, yText, "Match");
          }
        }
        else if(i==2) {
          if(Skills) {
            Brain.Screen.printAt(xText+5, yText, "A: %d", skillsAuton);
          }
          else {
            Brain.Screen.printAt(xText+5, yText, "A: %d", matchAuton);
          }
        }
        else if(i==3) {
          if(demo) {
            Brain.Screen.printAt(xText, yText, "Demo");
          }
          else if(programmingOveride) {
            Brain.Screen.printAt(xText, yText, "Prog OR");
          }
        }
      }
    }
    if(currentScreen == 3) {
      Brain.Screen.setFillColor(color::black);
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.drawRectangle(border, border, (width-6*border)/5, height-2*border);
      Brain.Screen.setFont(fontType::prop30);
      Brain.Screen.printAt(47, 70, "B");
      Brain.Screen.printAt(47, 110, "A");
      Brain.Screen.printAt(47, 150, "C");
      Brain.Screen.printAt(47, 190, "K");
      int rows;
      int font;
      int totalAutons;
      if(Skills) {
        totalAutons = totalSkillsAutons;
      }
      else {
        totalAutons = totalMatchAutons;
      }
      if(totalAutons < 3) {
        rows = 1;
        font = 60;
      }
      else if(totalAutons < 9) {
        rows = 2;
        font = 30;
      } 
      else if(totalAutons < 19) {
        rows = 3;
        font = 30;
      } 
      else {
        rows = 4;
        font = 20;
      } 
      int cols = (totalAutons+rows-.00001)/rows;
      Brain.Screen.setFillColor(color(100, 100, 100));
      for(int i = 0; i < totalAutons; i++) {
        int currentcol = i%cols;
        int currentrow = i/cols;
        int x = (width-6*border)/5 + border*(2+currentcol) + currentcol * (width-(cols+2)*border - (width-6*border)/5)/cols;
        int y = border*(1+currentrow) + currentrow * (height-(rows+1)*border)/rows;
        int w = (width-(cols+2)*border - (width-6*border)/5)/cols;
        int h = (height-(rows+1)*border)/rows;
        Brain.Screen.setPenColor(color(100, 100, 100));
        Brain.Screen.drawRectangle(x, y, w, h);
        Brain.Screen.setPenColor(color::white);
        if(font == 60) {
          Brain.Screen.setFont(fontType::prop60);
          Brain.Screen.printAt(x+w/2-15, y+h/2+15, "%d", i+1);
        }
        else if(font == 30) {
          Brain.Screen.setFont(fontType::prop30);
          if(i+1 < 10) {
            Brain.Screen.printAt(x+w/2-8, y+h/2+10, "%d", i+1);
          }
          else {
            Brain.Screen.printAt(x+w/2-16, y+h/2+10, "%d", i+1);
          }
        }
        else {
          Brain.Screen.setFont(fontType::prop20);
          if(i+1 < 10) {
            Brain.Screen.printAt(x+w/2-6, y+h/2+7, "%d", i+1);
          }
          else {
            Brain.Screen.printAt(x+w/2-11, y+h/2+7, "%d", i+1);
          }
        }
      }
    }
  }  //updates UI when screen is pressed
  void p(const char *s) {
    if(currentScreen != 4) {
      currentScreen = 4;
      Brain.Screen.clearScreen();
      Brain.Screen.setPenColor(color::white) ;
      Brain.Screen.setFillColor(color::black);
      Brain.Screen.setCursor(1, 1);
    }
    Brain.Screen.print(s);
  }  //displays data on screen
  void p(double n) {
    if(currentScreen != 4) {
      currentScreen = 4;
      Brain.Screen.clearScreen();
      Brain.Screen.setPenColor(color::white) ;
      Brain.Screen.setFillColor(color::black);
      Brain.Screen.setCursor(1, 1);
    }
    Brain.Screen.print(n);
  }
  void pl(const char *s) {
    if(currentScreen != 4) {
      currentScreen = 4;
      Brain.Screen.clearScreen();
      Brain.Screen.setPenColor(color::white) ;
      Brain.Screen.setFillColor(color::black);
      Brain.Screen.setCursor(1, 1);
    }
    Brain.Screen.print(s);
    Brain.Screen.newLine();
  }  //moves to next line as well
  void pl(double n) {
    if(currentScreen != 4) {
      currentScreen = 4;
      Brain.Screen.clearScreen();
      Brain.Screen.setPenColor(color::white) ;
      Brain.Screen.setFillColor(color::black);
      Brain.Screen.setCursor(1, 1);
    }
    Brain.Screen.print(n);
    Brain.Screen.newLine();
  }
  void cl(void) {
    if(currentScreen != 4) {
      currentScreen = 4;
      Brain.Screen.setPenColor(color::white) ;
      Brain.Screen.setFillColor(color::black);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  }  //clears screen