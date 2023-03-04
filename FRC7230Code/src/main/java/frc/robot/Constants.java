/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class robotConstants {
    public static final int L1MOTOR_ID = 1;
    public static final int R1MOTOR_ID = 3;
    public static final int L2MOTOR_ID = 2;
    public static final int R2MOTOR_ID = 4;
    
    public static final int BUTTON1 = 1; //X BUTTON: CLIMBER UP
    public static final int BUTTON2 = 2; //A BUTTON: CLIMBER DOWN
    public static final int BUTTON3 = 3; //B BUTTON: SMART VISION INTAKE
    public static final int BUTTON4 = 4; //Y BUTTON: REVERSE
    public static final int BUTTON5 = 5; //L BUMPER: SPEED
    public static final int BUTTON6 = 6; //R BUMPER: SLOW
    public static final int BUTTON7 = 7; //L TRIGGER: SHOOT
    public static final int BUTTON8 = 8; //R TRIGGER: INTAKE
    public static final int BUTTON9 = 9; //Button 9: BALANCING
    public static final int BUTTON10 = 10; //START BUTTON: RUN INTAKE MOTOR/NO SOLENOID
    public static final int BUTTON11 = 11;
    public static final int BUTTON12 = 12;

    //Drive Buttons:
    public static final int SPEED_BUTTON = BUTTON1;
    public static final int SLOW_BUTTON = BUTTON2;
    public static final int GAME_PIECE_TOGGLE_BUTTON = BUTTON5;
    public static final int MANUAL_SMART_TOGGLE_BUTTON = BUTTON4; //switching from smart and manual buttons
    public static final int BALANCING_BUTTON = BUTTON3;
    public static final int REVERSE_BUTTON = BUTTON6;

    //Manual Buttons:
    public static final int CLAW_TOGGLE_BUTTON = BUTTON8;
    public static final int SHELF_PICKUP_BUTTON = BUTTON10;
    public static final int GROUND_PICKUP_BUTTON = BUTTON12;
    public static final int HIGH_SCORE_BUTTON = BUTTON7;
    public static final int MID_SCORE_BUTTON = BUTTON9;
    public static final int LOW_SCORE_BUTTON = BUTTON11;

    //Smart Buttons:
    public static final int ORIENT_SHELF_PICKUP_BUTTON = BUTTON10;
    public static final int ORIENT_GROUND_PICKUP_BUTTON = BUTTON12;
    public static final int ORIENT_HIGH_TARGET_BUTTON = BUTTON7;
    public static final int ORIENT_MID_TARGET_BUTTON = BUTTON9;

    //Test Buttons:
    public static final int PID_TEST_BUTTON = BUTTON10;
    public static final int ENCODER_TEST_BUTTON = BUTTON11;
    public static final int EXTEND_TEST_BUTTON = BUTTON6;
    public static final int ARM_TEST_BUTTON_UP = BUTTON11;
    public static final int ARM_TEST_BUTTON_DOWN = BUTTON12;
  }
  
  public static final class driveTrainConstants {
    //drive related
    public static final double deadZone = 0.2;
    public static final double zoomFactor = 2.0;
    public static final double slowFactor = 0.45;
    public static final double speedFactor = 0.775;
    public static final double turnFactor = 0.725;
    public static final double accelY = 0.005;
    public static final double accelX = 0.001;
    public static final double decelY = 0.075;
    public static final double decelX = 0.075;
    public static final double limitX = 0.8;
    public static final double limitY = 0.6;
    public static final double initSpeed = 0.2;
    public static final double dropOff = 0.4; //drop off point when speed is able to drop to 0
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 3;
    public static final double kFF = 0.0015;

    //driveSetDistance/encoders
    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double motorRevsToWheelRevs = 1/10.71; //last year's
    // public static final double motorRevsToWheelRevs = 1/8.45; //this year's
    public static final double kEncoderMetersPerMotorRev = motorRevsToWheelRevs*kWheelDiameterMeters * Math.PI;
    public static final double driveMargin = 0.1;

    public static final double metersToPieceFromMiddle = 7.69;
    public static final double metersToPieceFromSide = 5.69;
    public static final double metersFromPieceToBalance = 1.25;
  
    public static final double testDistance = 1;

    //autobalance
    public static final double smartAngleMargin = 10;
    public static final double smartAngleMargin2 = 5;
    public static final double targetAngle = -67.33;
    public static final double smartSpeed = 0.55;
    public static final double slowSmartSpeed = 0.475;

    //vision
    public static final double smartAngleMarginVision = 3;
    public static final double smartSpeedVision = 0.55;
    public static final double[] coneScreenAreas = {4.5, 4.0}; // 0 - area of cone on the screen for ground pick up, 1 - for shelf pick up
    public static final double[] cubeScreenAreas = {5.0, 4.0}; // 0 for ground pick up, 1 for shelf pick up
    public static final double[] tapeScreenAreas = {0.12, 0.2}; // 0 - area of tape on the screen for high target, 1 - for middle target

    //arm-related
    public static final double armMotorSpeed = 0.5;
    public static final int rotationsToDegrees = 360;
    public static final double lowScoreAngleEncoderCounts = 152; //unextended
    public static final double shelfAngleEncoderCounts = 77; //extended
    public static final double lowPickupAngleEncoderCounts = -110; //extended
    public static final double armAngleMargin = 4;

    //cone
    public static final double coneHighAngleEncoderCounts = 83; //extended
    public static final double coneMidAngleEncoderCounts = 94; //unextended

    //cube
    public static final double cubeHighAngleEncoderCounts = 83; //extended
    public static final double cubeMidAngleEncoderCounts = 94; //unextended
    
    public static final double cubeShelfRetreatEncoderCounts = 0.25;
  }
}