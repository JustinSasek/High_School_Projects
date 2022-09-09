//Author:       7110A - Justin Sasek
//Created:      Sat Aug 26 2021
//Description:  Main Competition Code

//Settings///////////////////////////////////////////////////////////////////////////////////////////////////
  bool Red = true;  //Changes screen color and mogo ring automation
  bool Skills = false;  //Switches between skills and match autons
  bool demo = false;  //Locks wheels for interview
  bool programmingOveride = false;  //Runs programming skills instead of driver skills

  bool Reversed = false;  //Reverses drive train in user control and autonomous
  bool driveAcceleration = true;  //Turns on/off acceleration in matches

  int totalMatchAutons = 8;  //for UI
  int matchAuton = 4;  //current auton

  int totalSkillsAutons = 1;  //for UI
  int skillsAuton = 1;  //current programming skills

//Global Variables///////////////////////////////////////////////////////////////////////////////////////////

  #include "vex.h"
  using namespace vex;
  competition Competition;
  
  //Code
  int loopSleepTime = 10;  //msec  //sleep time for all continuous loops  //affects acceleration
  bool firstTimeRunning = true;

  //User Control Speeds
  double driveMaxSpeed = 100;
  double turnMinMaxSpeed = 40;  //turn speed decreases when holding goal
  double turnMaxMaxSpeed = 60;
  double liftMaxSpeed = 100;
  double middleWheelRetractionSpeed = 40;
  double driveSpeed = 0;

  //User Control Acceleration  //How many times the code repeats until the desired speed is met
  int minForwardDriveCycles = 42;  //drive acceleration increases when holding goal
  int maxForwardDriveCycles = 50;
  int maxBackwardDriveCycles = 100;
  int turnCycles = 40;
  int liftCycles = 20;
  int ringIntakeCycles = 1;

  //Auton Speeds
  double aDriveMaxSpeed = 50;  //min and max speeds for trapesoidal velocity
  double aDriveMinSpeed = 15;
  double aTurnMaxSpeed = 50;
  double aTurnMinSpeed = 6;
  double aLiftMaxSpeed = 60;
  double aLiftMinSpeed = 20;
  double aLiftOffsetSpeed = 0;
  double aRingIntakeMaxSpeed = 75; 

  //Auton Acceleration
  int aDriveCycles = 150;  
  int aTurnCycles = 150;
  int aLiftCycles = 20; 
  int aRingIntakeCycles = 1;

  //Auton Decceleration
  double aDriveDecel = 20;  //from max to min speed in inches
  double aTurnDecel = 70;  //from max to min speed in degrees
  double aLiftDecel = 60;  //from max to min speed in degrees
  double lockCourseDistance = 6;  //in  //distance from destination where robot stops turning

  //Drivetrain Calibration
  double degreesperinch = 21.8;  //motor degree per robot inch
  double degreesperdegree = 2.67;  //motor degrees per robot degree

  //Sensor Thresholds
  int liftTolerance = 5;  //Distance from lift position where lift motors lock
  int redColorThreshold = 40;  //Color on sensor to intake rings  //17
  int blueColorThreshold = 140;  //213  //yellow 68  nothing: 64
  int ringSensorThreshold = 0;  //Distance on sensor to intake rings

  //Positions
  int initialLiftPosition = 7;  //Lifts lift more the first time
  int accelerationLiftPosition = 109;  //Increases acceleration and limits turn speed past this point
  int liftLevel [5] = {24, 150, 280, 315, 504};  //automated lift positions for grabing goals
  int ringIntakeScoreLength = 500;  //Degrees to hold a ring
  int ringIntakeHoldLength = 400;  //Degrees to score a ring
  int middleWheelRetractionDistance = 40;  //How far to retract the middle wheel when not in use

  //Delays
  int releaseDelay = 300;  //Waits this time before releasing clamp to let lift lower
  int clampSensorDelay = 1000;  //Waits this time before reactivating the clamp sensor
  int mogoIntakeSensorDelay = 3000;  //Waits this time before reactivting the mogo Sensor
  int aBrakeTime = 75;  //msec  //time to brake after moving in auton

//

void pre_auton(void) {
  vexcodeInit();
  UIInit();

  Brain.Screen.pressed(updateUI);
  Controller.ButtonL1.pressed(switchLift);  //only works with updateLiftLevels on
  Controller.ButtonL2.pressed(switchClamp);
  Controller.ButtonR2.pressed(switchMogoIntake);
  Controller.ButtonA.pressed(reverseDriving);
  Controller.ButtonB.pressed(switchRingIntake);
  Controller.ButtonX.pressed(switchDriveAccleration);
  //Controller.ButtonY.pressed(toggleMiddleWheel);
  clampSensor.pressed(clampSensorPressed);

  rfDrive.resetPosition();
  rbDrive.resetPosition();
  lfDrive.resetPosition();
  lbDrive.resetPosition();
  middleDrive.resetPosition();
  rLift.resetPosition();
  lLift.resetPosition();
  ringIntake.resetPosition();

  mogoIntakeSensor.setLight(ledState::on);
  wait(5, msec);
  Brain.Timer.reset();
}

void autonomous(void) {
  //Pre Loop
    int autonStartTime = Brain.Timer.time();
    rfDrive.resetPosition();
    rbDrive.resetPosition();
    lfDrive.resetPosition();
    lbDrive.resetPosition();
    rLift.resetPosition();
    lLift.resetPosition();

    firstTimeRunning = false;
    updateLift = false;
    updateLiftLevels = true;
    updateClampSensor = false;
    updateMogoIntakeSensor = false;
    updateMogoIntakeColorSensor = false;
    updateMiddleWheel = false;;
    Reversed = false;
    trackingWheels = false;
    releaseDelay = 0;
    aDriveMaxSpeed = 50;
    aDriveDecel = 20;
    aDriveCycles = 150;
    degreesperdegree = 2.67; 
    aTurnMaxSpeed = 50;
    aTurnDecel = 70;
    aTurnCycles = 150;
    aLiftMaxSpeed = 60;
    aBrakeTime = 75;
    currentLiftLevel = 1;
    clamped = true;
    intakingRings = false;
    aMogoOuttake();
    currentX = 0;
    currentY = 0;
    currentRot = 0;
    currentLeft = 0;
    currentRight = 0;
  //Skills////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(Skills && skillsAuton == 1) {
      Reversed = true;
      int longMoveSpeed = 80;
      degreesperdegree = 2.42666666667;
      aLiftMaxSpeed = 30;
      updateLift = true;
      updateLiftLevels = false;
      aForward(23);
      aLiftMaxSpeed = 60;
      aOpenClamp();
      updateLiftLevels = true;
      currentLiftLevel = 0;
      aMogoIntake();  //left red
      aWait(.3);
      aForward(-4.6);
      aTurn(95);
      updateClampSensor = true;
      Reversed = false;
      aDriveMaxSpeed = longMoveSpeed;
      aForward(57);
      updateClampSensor = false;
      aCloseClamp();  //left nuetral
      intakingRings = true;
      aTurn(28);
      aRaiseLift(390);
      aForward(43);
      aDriveMaxSpeed = 50;
      aRaiseLift(250);
      aWait(0.7);
      aOpenClamp();  //left nuetral
      aForward(-5);

      aTurn(-133);
      intakingRings = false;
      aMogoOuttake();  //red
      aLowerLift();
      aWait(.5);
      aForward(15.1);
      aTurn(190);
      updateClampSensor = true;
      aForward(15.1);
      aCloseClamp();  //red
      aRaiseLift(316);
      aTurn(-65);
      aWait(.1);

      aForward(15);
      aOpenClamp();  //red
      aWait(.4);
      aForward(-12);
      aLowerLift();
      aTurn(-121);
      updateClampSensor = true;
      aDriveMaxSpeed = longMoveSpeed;
      aForward(39.7, 2.1);
      aDriveMaxSpeed = 50;
      aCloseClamp();
      updateClampSensor = false;
      aRaiseLift();
      aForward(-3);
      aWait(.1);
      aDriveMaxSpeed = longMoveSpeed;
      aForward(20.8, 1.5);  //2
      aDriveMaxSpeed = 50;

      //Reset  11111111111111111111111111111111111111

      Reversed = true;
      aForward(14);
      aTurn(31);
      aLiftMaxSpeed = 30;
      aLowerLift();
      aForward(70);
      aLiftMaxSpeed = 60;
      aDriveMaxSpeed = longMoveSpeed;
      aMogoIntake();  //tall nuetral
      aWait(.5);
      aTurn(11);
      aForward(58);
      aDriveMaxSpeed = 50;
      aMogoOuttake();  //tall nuetral
      aForward(-14);
      aTurn(-45);
      aForward(17.8, 1.5);  //2
      aMogoIntake();  //red alliance
      intakingRings = true;
      aWait(.3);
      aRaiseLift();
      Reversed = false;
      aDriveMaxSpeed = longMoveSpeed;
      aForward(52);
      aDriveMaxSpeed = 50;
      aTurn(-100);
      intakingRings = false;
      aForward(5);
      
      aOpenClamp();
      aForward(-3.2);
      aRaiseLift(167);
      aWait(0.3);
      aForward(5.5, 1.1);

      //Reset  222222222222222222222222222222222222222
      
      aForward(-8.8);
      aTurn(-133); //-131
      intakingRings = true;
      updateClampSensor = true;
      aForward(23);
      aLowerLift();
      aWait(.2);
      aForward(19);
      aCloseClamp();  //right nuetral
      updateClampSensor = true;
      aTurn(-89.6);
      aRaiseLift(312);
      aDriveMaxSpeed = longMoveSpeed;
      aForward(51, 2.7);
      aDriveMaxSpeed = 50;
      aOpenClamp();
      aWait(.5);
      aDriveMaxSpeed = 100;
      aForward(-10.5);
      aTurn(-60);
      aForward(39);
      intakingRings = false;
      aMogoOuttake();
      aForward(12);

    }
    if(Skills && skillsAuton == 2) {
      Reversed = true;
      trackingWheels = true;
      int longMoveSpeed = 80;
      //degreesperdegree = 2.77; 
      aLiftMaxSpeed = 30;
      updateLift = true;
      updateLiftLevels = false;
      aForward(23);
      aLiftMaxSpeed = 60;
      aOpenClamp();
      updateLiftLevels = true;
      currentLiftLevel = 0;
      aMogoIntake();
      aWait(.3);
      aTurn(16);
      aForward(-4.6);
      aTurn(81);  //79
      updateClampSensor = true;
      Reversed = false;
      aDriveMaxSpeed = longMoveSpeed;
      aForward(58);
      updateClampSensor = false;
      aCloseClamp();  //nuetral goal
      intakingRings = true;
      aTurn(20);
      aRaiseLift(390);
      aForward(43);
      aDriveMaxSpeed = 50;
      aRaiseLift(250);
      aWait(0.7);
      aOpenClamp();
      aForward(-5);

      aTurn(-133);
      intakingRings = false;
      aMogoOuttake();
      aLowerLift();
      aWait(.5);
      aForward(15.1);
      aTurn(190);
      updateClampSensor = true;
      aForward(20);
      aCloseClamp();  //red goal
      aWait(.5);
      aRaiseLift(302);
      aTurn(-65);
      aWait(.1);

      aForward(5);
      aOpenClamp();  //red goal
      aWait(.4);
      aForward(-7);
      aLowerLift();
      aTurn(-93.8);
      updateClampSensor = true;
      aDriveMaxSpeed = longMoveSpeed;
      aForward(41.6, 2.1);
      aDriveMaxSpeed = 50;
      aCloseClamp();  //blue
      updateClampSensor = false;
      aRaiseLift();
      aForward(-3);
      aWait(.1);
      aDriveMaxSpeed = longMoveSpeed;
      aForward(20, 2);  
      aDriveMaxSpeed = 50;

      //Reset  11111111111111111111111111111111111111

      aDriveCycles = 250;
      aTurnMaxSpeed = 35;
      aLiftMaxSpeed = 30;
      Reversed = true;
      aForward(14);
      aTurn(42);
      aLowerLift();
      aForward(70);
      aDriveMaxSpeed = longMoveSpeed;
      aMogoIntake();
      aWait(.5);
      aTurn(9);
      aForward(57);  //56
      aDriveMaxSpeed = 50;
      aMogoOuttake();  //tall nuetral
      aDriveCycles = 150;
      aTurnMaxSpeed = 50;
      aLiftMaxSpeed = 60;
      aForward(-14);
      aTurn(-44);
      aForward(18.5, 2);
      aMogoIntake();  //red
      intakingRings = true;
      aWait(.3);
      aRaiseLift();
      Reversed = false;
      aDriveMaxSpeed = longMoveSpeed;
      aForward(49);
      aDriveMaxSpeed = 50;
      aTurn(-87);
      intakingRings = false;
      aForward(5);
      
      aOpenClamp();  //blue
      aForward(-3.2);
      aRaiseLift(187);
      aWait(0.3);
      aForward(2.5, 1.1);

      //Reset  222222222222222222222222222222222222222
      
      aForward(-8.8);
      aTurn(-128); //-136
      intakingRings = true;
      updateClampSensor = true;
      
      aForward(18);
      aLowerLift();
      aWait(.2);
      aForward(11);
      aCloseClamp();
      updateClampSensor = true;
      aTurn(-79);
      aRaiseLift(312);
      aDriveMaxSpeed = longMoveSpeed;
      aForward(51, 3.5);
      aDriveMaxSpeed = 50;
      aOpenClamp();
      aWait(.5);
      aDriveMaxSpeed = 100;
      aForward(-10.5);
      aTurn(-60);
      aMogoOuttake();
      intakingRings = false;
      aForward(39);

    }
  //Match////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(!Skills && matchAuton == 1) {
      Reversed = true;
      degreesperdegree = 2.42666666667;
      aLiftMaxSpeed = 30;
      updateLift = true;
      updateLiftLevels = false;
      aForward(23);
      aLiftMaxSpeed = 60;
      aOpenClamp();
      updateLiftLevels = true;
      currentLiftLevel = 0;
      aMogoIntake();
      aWait(.3);
      intakingRings = true;
      aRaiseLift(100);
      aForward(-24);
    }  //1/2 auton wp
    if(!Skills && matchAuton == 2) {
      aRaiseLift(150);
      updateLift = true;
      aForward(18.4);
      aOpenClamp();
      aForward(-3.7);
      aRaiseLift(373);
      aTurn(-90);
      aForward(17.3);
      Reversed = true;
      aTurn(-88.5);
      aForward(103, 5);
      aMogoIntake();
      aWait(.8);
      intakingRings = true;
      aForward(-27);   
      aMogoOuttake();    
      aForward(-6);    
      
      
      /*Reversed = true;
      aForward(27);
      aRaiseLift();
      aMogoIntake();
      updateLift = true;
      aWait(0.75);
      intakingRings = true;

      aTurn(95);
      Reversed = false;

      aForward(98);
      aLowerLift();
      aTurn(-124);
      aForward(17);
      intakingRings = false;
      releaseDelay = 0;
      updateLift = false;
      aOpenClamp();
      aWait(0.5);
      aForward(-6);
      aTurn(-135);
      aMogoOuttake();
      aForward(-5);
      aForward(10);*/
    }  //full auton wp
    if(!Skills && matchAuton == 3) {
      aDriveMaxSpeed = 65;
      aTurnDecel = 90;
      aTurnMaxSpeed = 65;
      aRaiseLift(150);
      updateLift = true;
      //aForward(18.4);
      aForward(7.5);
      aOpenClamp();  //ring
      aForward(-3.7);
      aTurn(-89.8);
      aForward(19.3);
      Reversed = true;
      aTurn(-89.5);
      aDriveCycles = 220;
      aDriveMaxSpeed = 100;
      aForward(100, 3);
      aMogoIntake();  //blue
      aWait(.4);
      intakingRings = true;
      Reversed = false;
      aDriveMaxSpeed = 65;
      aForward(21);
      aTurn(90);
      aLowerLift();
      intakingRings = false;
      updateClampSensor = true;
      aBrakeTime = 0;
      aForward(32);
      aCloseClamp();
      aDriveMaxSpeed = 100;
      aDriveCycles = 1;
      aDriveDecel = 10;
      intakingRings = true;
      aForward(-48);
      aMogoOuttake();
      intakingRings = false;
      aForward(4);         
    }  //full auton wp + GR
    if(!Skills && matchAuton == 4) {
        aDriveMaxSpeed = 100;
        aOpenClamp();
        updateLift = true;
        updateClampSensor = true;
        
        aForward(51);
        aCloseClamp();
        aForward(-33);
        updateLiftLevels = true;
        aTurn(-81);
        aDriveMaxSpeed = 60;
        aForward(-16, 1.2);
        aDriveMaxSpeed = 100;
        aMogoIntake();
        aWait(.3);

        aRaiseLift(185);
        aTurn(18.5);
        intakingRings = true;
        aForward(30);
        
        aTurn(-18);
        aDriveMaxSpeed = 45;
        aForward(53);
        aWait(0.3);
        aDriveMaxSpeed = 100;
        aTurn(26);
        aLowerLift();
        aForward(-24);
        aMogoOuttake();
        aForward(5);



        /*
        aForward(15);
        aTurn(-140);

        aRaiseLift();
        aDriveMaxSpeed = 70;
        aForward(30); //manual ring loading
        aForward(-30);
        aForward(30);
        aForward(-20);
        aMogoOuttake();
        aLowerLift();
        aForward(10);
        intakingRings = false;*/



    }  //goal rush, right alliance, flowers
    if(!Skills && matchAuton == 5) {
        updateLift = true;
        aDriveMaxSpeed = 60;
        aOpenClamp();
        aForward(32);
        aTurn(-55.4);
        updateClampSensor = true;
        aForward(38.9);
        aForward(-61);
        aMogoIntake();
        aForward(6.7);
        intakingRings = true;
        aTurn(68);
        aDriveMaxSpeed = 40;
        aForward(32);
        aWait(0.7);
        aDriveMaxSpeed = 100;
        aDriveCycles = 1;
        aDriveDecel = 12;
        aBrakeTime = 0;
        aForward(-50);
        intakingRings = false;
        aMogoOuttake();
        aForward(6);

    }  //middle goal rush, right alliance, flowers
    if(!Skills && matchAuton == 6) {
        aDriveMaxSpeed = 100;
        aOpenClamp();
        updateLift = true;
        updateClampSensor = true;
        
        aForward(51);
        aCloseClamp();
        aForward(-31);
        updateLiftLevels = true;
        aTurn(-86);
        aDriveMaxSpeed = 60;
        aForward(-16, 1.2);
        aDriveMaxSpeed = 100;
        aMogoIntake();
        aWait(.3);

        aRaiseLift(185);
        aTurn(18.5);
        intakingRings = true;
        aForward(34);

        aTurn(21.6);
        aForward(-48.5);
        
        /*aTurn(-31);
        aDriveMaxSpeed = 35;
        aForward(45);
        aWait(1.2);
        aDriveMaxSpeed = 100;
        aTurn(8);
        aMogoOuttake();
        aLowerLift();
        aForward(-10);
        aForward(5);*/
    }  //goal rush, right alliance
    if(!Skills && matchAuton == 7) {
        aDriveMaxSpeed = 100;
        aDriveCycles = 1;
        aDriveDecel = 1;
        aBrakeTime = 0;
        aOpenClamp();
        updateLift = true;
        updateLiftLevels = false;
        updateClampSensor = true;
        
        aForward(51);
        aCloseClamp();
        aForward(-63360);
    }  //goal rush
    if(!Skills && matchAuton == 8) {
    }  //Blank
    if(!Skills && matchAuton == 9) {
      updateLift = true;
      aRaiseLift();
      aWait(2);
      aForward(34);
      aTurn(90);
      updateClampSensor = true;
      aForward(10);
      aCloseClamp();
    }
    if(!Skills && matchAuton == 10) {
      aDriveMaxSpeed = 100;
      aDriveDecel = 20;
      aDriveCycles = 250;

      aTurnMaxSpeed = 200;
      aTurnDecel = 45;
      aTurnCycles = 250;
      aGoForwardTo(48.8, -23.53);
    }  //odom
  Controller.Screen.clearScreen();  //Displays auton stop time
  Controller.Screen.setCursor(1, 0);
  Controller.Screen.print("Auton Time: ");
  Controller.Screen.print((double)(Brain.Timer.time() - autonStartTime)/1000);
  Controller.Screen.print("sec");
  aWait(100);
}

void usercontrol(void) {
  //Preloop////////////////////////////////////////////////////////////////////////////////////////////////////
    if(programmingOveride) {
      autonomous();
      while (1) {
        wait(1, sec);
      }
    }
    if(firstTimeRunning) {
      rLift.resetPosition();
      lLift.resetPosition();
    }

    firstTimeRunning = false;
    updateLift = false;
    updateLiftLevels = false;
    updateClampSensor = true;
    updateMogoIntakeSensor = false;
    updateMiddleWheel = false;
    Reversed = false;
    releaseDelay = 0;
    
    double turnSpeed = 0;
    double liftSpeed = 0;

    bool rumbled[3] = {0, 0, 0};
    int driverStartTime = Brain.Timer.time();

    while (1) {
    
    double driveTargetSpeed = 0;
    double turnTargetSpeed = 0;
    double liftTargetSpeed = 0;
  //Target Variables///////////////////////////////////////////////////////////////////////////////////////////
    //axis 3 goes forward and backwards
    //axis 1 turns left and right
    //button A switches driving
    //button B switches ring conveyor
    //button X toggles acceleration
    //button L1 raises/toggles lift
    //button R1 lowers lift
    //button L2 toggles clamp
      int currentLiftPosition=rLift.position(deg) + lLift.position(deg);
      currentLiftPosition /= 2;
    
    //Drive Train
      driveTargetSpeed = driveMaxSpeed * Controller.Axis3.position() / 100;
      if(Reversed) {
        driveTargetSpeed *= -1;
      }

      int turnMaxSpeed = turnMaxMaxSpeed;
      if(currentLiftPosition > accelerationLiftPosition && currentLiftPosition < liftLevel[4]) {
        int n = (turnMaxMaxSpeed - turnMinMaxSpeed) * (accelerationLiftPosition - currentLiftPosition) / (accelerationLiftPosition - liftLevel[4]);
        if(!clamped) {
          n /= 3;
        }
        turnMaxSpeed -= n;
      }
      turnTargetSpeed = turnMaxSpeed * Controller.Axis1.position() / 100;

      //Control Remapping
      double temp;
      temp = pow(driveTargetSpeed / 100, 2) * 100;
      if(driveTargetSpeed < 0) {
        temp *= -1;
      }
      temp = .6*driveTargetSpeed + .4*temp;
      driveTargetSpeed = temp;

      temp = pow(turnTargetSpeed / 100, 2) * 100;
      if(turnTargetSpeed < 0) {
        temp *= -1;
      }
      temp = .6*turnTargetSpeed + .4*temp;
      turnTargetSpeed = temp;      
  
    //Lift
      if(Controller.ButtonL1.pressing()) {
        liftTargetSpeed += liftMaxSpeed; 
      }
      if(Controller.ButtonR1.pressing() && currentLiftPosition > liftLevel[0] + liftTolerance + 25) {
        liftTargetSpeed -= liftMaxSpeed-40; 
      }
      if(!updateLiftLevels) {
        if(currentLiftPosition < liftLevel[0] - liftTolerance) {
          updateLift = true;
        }
        else {
          updateLift = false;
        }
      }
      
      
    //Ring Intake
          

  //Acceleration///////////////////////////////////////////////////////////////////////////////////////////////
    //Caps acceleration for smoother movements and less tipping
    if(driveAcceleration) {
      int forwardDriveCycles = 1;
      int backwardDriveCycles = 1;
      if(currentLiftPosition > accelerationLiftPosition) {
        forwardDriveCycles += maxForwardDriveCycles - (maxForwardDriveCycles - minForwardDriveCycles) 
          * (currentLiftPosition - accelerationLiftPosition) / (liftLevel[4] - accelerationLiftPosition);
      }
      if(currentLiftPosition > liftLevel[0]) {
        backwardDriveCycles += (maxBackwardDriveCycles - 1) * currentLiftPosition / liftLevel[3];
      }
      if(!clamped) {
        forwardDriveCycles /= 3;
        turnCycles /= 3;
      }
      
      double driveAcceleration = driveTargetSpeed - driveSpeed;
      if (driveAcceleration < -2 * driveMaxSpeed / forwardDriveCycles) {
        driveAcceleration = (-2 * driveMaxSpeed / forwardDriveCycles);
      }
      if (driveAcceleration > 2 * driveMaxSpeed / backwardDriveCycles) {
        driveAcceleration = (2 * driveMaxSpeed / backwardDriveCycles);
      }
      driveSpeed += driveAcceleration;
      
      double turnAcceleration = turnTargetSpeed - turnSpeed;
      if (turnAcceleration > 2 * turnMaxSpeed / turnCycles) {
        turnAcceleration = (2 * turnMaxSpeed / turnCycles);
      }
      else if (turnAcceleration < -2 * turnMaxSpeed / turnCycles) {
        turnAcceleration = (-2 * turnMaxSpeed / turnCycles);
      }
      turnSpeed += turnAcceleration;
    }
    else {
      driveSpeed = driveTargetSpeed;
      turnSpeed = turnTargetSpeed;
    }
      
    double liftAcceleration = liftTargetSpeed - liftSpeed;
    if (liftAcceleration > 2 * liftMaxSpeed / liftCycles) {
      liftAcceleration = (2 * liftMaxSpeed / liftCycles);
    }
    else if (liftAcceleration < -2 * liftMaxSpeed / liftCycles) {
      liftAcceleration = (-2 * liftMaxSpeed / liftCycles);
    }
    liftSpeed += liftAcceleration;

  //Constrain//////////////////////////////////////////////////////////////////////////////////////////////////
    //Limits drive and turn speed porportionally when higher than 100%
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
    
  //Implement//////////////////////////////////////////////////////////////////////////////////////////////////
    //Spins motors and desired speeds
    static int lastDriveStillTime = Brain.Timer.time();   
    
    bool driveStill = (
      abs((int)driveSpeed) < 1 
      && abs((int)turnSpeed) < 1 
      && abs(Controller.Axis3.position()) < 1 
      && abs(Controller.Axis1.position()) < 1
    );
    if(!driveStill) {
      lastDriveStillTime = Brain.Timer.time(); 
      rfDrive.spin(forward, rf, percent);
      rbDrive.spin(forward, rb, percent);
      lfDrive.spin(forward, lf, percent);
      lbDrive.spin(forward, lb, percent);
    } 
    else if(Brain.Timer.time() - lastDriveStillTime <= 1000) {
      rfDrive.stop(brakeType::brake);
      rbDrive.stop(brakeType::brake);
      lfDrive.stop(brakeType::brake);
      lbDrive.stop(brakeType::brake);
    }
    else {
      rfDrive.stop(brakeType::coast);
      rbDrive.stop(brakeType::coast);
      lfDrive.stop(brakeType::coast);
      lbDrive.stop(brakeType::coast);
    }
    if(demo) {
      updateLift = true;
      updateLiftLevels = true;
      updateMogoIntakeSensor = true;
      updateMiddleWheel = true;
      releaseDelay = 300;
    }
    else {
      updateLift = false;
      updateLiftLevels = false;
      updateMogoIntakeSensor = false;
      updateMiddleWheel = false;
      releaseDelay = 0;
    }

    if(!updateLift) {
      bool liftStill = (
        abs((int)liftSpeed) < 1 
        && abs((int)liftTargetSpeed) < 1
      );
      if(!liftStill) {
        rLift.spin(forward, liftSpeed, percent);
        lLift.spin(forward, liftSpeed, percent);
      } 
      else {
        rLift.stop(brakeType::hold);
        lLift.stop(brakeType::hold);
      }
    }

    //Rumble - to let driver know how much time is left
    double remainingTime = 60 - (double)(Brain.Timer.time() - driverStartTime)/1000;
    if(!Skills) {
      remainingTime += 45;
    }
    if(!rumbled[0]) {
      if(Skills) {
        if(remainingTime <= 30) {
          Controller.rumble(".");
          rumbled[0] = 1;
        }
      }
      else {
        if(remainingTime <= 40) {
          Controller.rumble(".");
          rumbled[0] = 1;
        }
      }
    }
    else if(!rumbled[1]) {
      if(remainingTime <= 5) {
        Controller.rumble(".");
        rumbled[1] = 1;
      }
    }
    else if(!rumbled[2]) {
      if(remainingTime <= 1) {
        Controller.rumble("-");
        rumbled[2] = 1;
        aMogoOuttake();  //to avoid transitive property
      }
    }
    
    
  //Measure////////////////////////////////////////////////////////////////////////////////////////////////////
    //Allows for easy auton creation
    //You can drive the robot around and measure distances then input the values into the auton functions
    if(Controller.ButtonLeft.pressing()) {
      double n;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 0);
        Controller.Screen.print("Fd: ");
        n = rfDrive.position(deg) + rbDrive.position(deg) + lfDrive.position(deg) + lbDrive.position(deg);
        if(Reversed) {
          n*=-1;
        }
        Controller.Screen.print(n / 4 / degreesperinch);
        Controller.Screen.print(" ");
        Controller.Screen.print("Tn: ");
        n = -1*rfDrive.position(deg) + -1*rbDrive.position(deg) + lfDrive.position(deg) + lbDrive.position(deg);
        Controller.Screen.print(n / 4 / degreesperdegree);
        Controller.Screen.print("                  ");
      Controller.Screen.newLine();

        Controller.Screen.print("Lft: ");
        n = rLift.position(deg) + lLift.position(deg);
        Controller.Screen.print(n/2);
        Controller.Screen.print(" ");
        Controller.Screen.print("Rot: ");
        Controller.Screen.print(currentRot);
        Controller.Screen.print("                  ");
      Controller.Screen.newLine();

        Controller.Screen.print("X: ");
        Controller.Screen.print(currentX);
        Controller.Screen.print(" ");
        Controller.Screen.print("Y: ");
        Controller.Screen.print(currentY);
        Controller.Screen.print("                  ");
    }

    if(Controller.ButtonRight.pressing()) {
      rfDrive.resetPosition();
      rbDrive.resetPosition();
      lfDrive.resetPosition();
      lbDrive.resetPosition();
      ringIntake.resetPosition();
      currentX = 0;
      currentY = 0;
      currentRot = 0;
      currentLeft = 0;
      currentRight = 0;
    }
    
    aUpdateAll();  //Runs all update functions
    wait(loopSleepTime, msec);
    }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
