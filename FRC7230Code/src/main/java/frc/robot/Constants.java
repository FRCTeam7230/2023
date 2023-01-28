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

    public static final int CLIMBER_UP_BUTTON = BUTTON10;
    public static final int CLIMBER_DOWN_BUTTON = BUTTON12;
    public static final int SMART_INTAKE_BUTTON = BUTTON2;
    public static final int REVERSE_BUTTON = BUTTON4;
    public static final int SPEED_BUTTON = BUTTON5;
    public static final int SLOW_BUTTON = BUTTON6;
    public static final int SHOOT_BUTTON = BUTTON7;
    public static final int INTAKE_BUTTON = BUTTON8;
    public static final int INTAKE_MOTOR_BUTTON = BUTTON10;
    public static final int BALANCING_BUTTON = BUTTON9;
    public static final int SMART_ORIENT_BUTTON = BUTTON11;

    public static final double shootingPower = 0.6;
    public static final double climbPower = 0.6;

    public static final int cameraWidth = 160;
    public static final int cameraLength = 120;
  }
  
  public static final class driveTrainConstants {
    public static final double deadZone=0.3;
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
    public static final double smartAngleMargin = 3;
    public static final double targetAngle = 0;
    public static final double smartSpeed = 0.45;
    public static final double dropOff = 0.4; //drop off point when speed is able to drop to 0
    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
  }
}