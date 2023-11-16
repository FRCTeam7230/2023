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
    public static final int ORIENT_BUTTON = BUTTON9;

    //Test Buttons:
    // public static final int PID_TEST_BUTTON = BUTTON10;
    // public static final int ENCODER_TEST_BUTTON = BUTTON11;
    // drive stick











    public static final int CLOSE_ARM_BUTTON = BUTTON7;
    public static final int EXTEND_TEST_BUTTON = BUTTON10;
    public static final int ARM_TEST_BUTTON_UP = BUTTON11;
    public static final int ARM_TEST_BUTTON_DOWN = BUTTON12;
  }
  
  public static final class driveTrainConstants {
    //drive related
    public static final double deadZone = 0.2;
    public static final double zoomFactor = 2.0;
    public static final double slowFactor = 0.45;
    public static final double speedFactor = 0.575;
    public static final double turnFactor = 0.725;
    public static final double accelY = 0.005;
    public static final double accelX = 0.001;
    public static final double decelY = 0.075;
    public static final double decelX = 0.075;
    public static final double limitX = 0.7;
    public static final double limitY = 0.45;
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

    public static final double metersToPieceFromMiddle = 4.5;
    public static final double metersToPieceFromSide = 4.5;
    public static final double metersFromPieceToBalance = 1.25;
  
    public static final double testDistance = 1;

    //autobalance
    public static final double smartAngleMargin = 3;
    public static final double smartAngleMargin2 = 4;
    public static final double smartAngleMargin3 = 2;
    public static final double targetAngle = -0.75;
    public static final double smartSpeed = 0.45;
    public static final double slowSmartSpeed = 0.45;
    public static final double slowerSmartSpeed = 0.3;
    public static final double metersToCenterofBalance = 1;

    //vision
    public static final double smartAngleMarginVision = 3;
    public static final double smartAngleMarginVisionArm = 1.5;
    public static final double smartSpeedVision = 0.55;
    public static final double[] coneScreenAreas = {0.8, 0.8}; // 0 - area of cone on the screen for ground pick up, 1 - for shelf pick up
    public static final double[] cubeScreenAreas = {1.0, 1.0}; // 0 for ground pick up, 1 for shelf pick up
    public static final double[] tapeScreenAreas = {0.12, 0.2}; // 0 - area of tape on the screen for high target, 1 - for middle target
    public static final double[] aprilTagScreenAreas = {1,1};
    public static final double coneVertOffsetAngle = 0;
    public static final double cubeVertOffsetAngle = 3.5;
    public static final double tapeOffsetAngle = 0;
    public static final double tagOffsetAngle = 0;

    //arm-related
    public static final double drift = 115;
    public static final double armLowerLimit = -67+drift;
    public static final double armLowerExtension = 20+drift;
    public static final double armUpperLimit = 195+drift;
    public static final double armUpperRetraction = 60+drift;
    public static final double armMotorSpeed = 0.55;
    public static final int rotationsToDegrees = 360;
    public static final double lowScoreAngleEncoderCounts = 180+drift; //unextended
    public static final double shelfAngleEncoderCounts = -20  +drift; //extended
    public static final double lowPickupAngleEncoderCounts = armLowerLimit+7; //extended
    public static final double armAngleMargin = 3.5;
    public static final double groundPickupAngleMargin = 5;


    //cone
    public static final double coneHighAngleEncoderCounts = 115+drift; //extended
    public static final double coneMidAngleEncoderCounts = 123+drift; //unextended

    //cube
    public static final double cubeHighAngleEncoderCounts = coneHighAngleEncoderCounts+10; //extended
    public static final double cubeMidAngleEncoderCounts = coneMidAngleEncoderCounts+10; //unextended
  }
}
// /*// Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide
//  * numerical or boolean
//  * constants. This class should not be used for any other purpose. All constants
//  * should be declared
//  * globally (i.e. public static). Do not put anything functional in this class.
//  *
//  * <p>
//  * It is advised to statically import this class (or one of its inner classes)
//  * wherever the
//  * constants are needed, to reduce verbosity.
//  */
// public final class Constants {
//   public static final class DriveConstants {
//     // Driving Parameters - Note that these are not the maximum capable speeds of
//     // the robot, rather the allowed maximum speeds
//     public static final double kMaxSpeedMetersPerSecond = 4.8;
//     public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

//     public static final double kDirectionSlewRate = 1.2; // radians per second
//     public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
//     public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

//     // Chassis configuration
//     public static final double kTrackWidth = Units.inchesToMeters(26.5);
//     // Distance between centers of right and left wheels on robot
//     public static final double kWheelBase = Units.inchesToMeters(26.5);
//     // Distance between front and back wheels on robot
//     public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//         new Translation2d(kWheelBase / 2, kTrackWidth / 2),
//         new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
//         new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
//         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

//     // Angular offsets of the modules relative to the chassis in radians
//     public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
//     public static final double kFrontRightChassisAngularOffset = 0;
//     public static final double kBackLeftChassisAngularOffset = Math.PI;
//     public static final double kBackRightChassisAngularOffset = Math.PI / 2;

//     // SPARK MAX CAN IDs
//     public static final int kFrontLeftDrivingCanId = 11;
//     public static final int kRearLeftDrivingCanId = 13;
//     public static final int kFrontRightDrivingCanId = 15;
//     public static final int kRearRightDrivingCanId = 17;

//     public static final int kFrontLeftTurningCanId = 10;
//     public static final int kRearLeftTurningCanId = 12;
//     public static final int kFrontRightTurningCanId = 14;
//     public static final int kRearRightTurningCanId = 16;

//     public static final boolean kGyroReversed = false;
//   }

//   public static final class ModuleConstants {
//     // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
//     // This changes the drive speed of the module (a pinion gear with more teeth will result in a
//     // robot that drives faster).
//     public static final int kDrivingMotorPinionTeeth = 14;

//     // Invert the turning encoder, since the output shaft rotates in the opposite direction of
//     // the steering motor in the MAXSwerve Module.
//     public static final boolean kTurningEncoderInverted = true;

//     // Calculations required for driving motor conversion factors and feed forward
//     public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
//     public static final double kWheelDiameterMeters = 0.0762;
//     public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
//     // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
//     public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
//     public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
//         / kDrivingMotorReduction;

//     public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
//         / kDrivingMotorReduction; // meters
//     public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
//         / kDrivingMotorReduction) / 60.0; // meters per second

//     public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
//     public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

//     public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
//     public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

//     public static final double kDrivingP = 0.04;
//     public static final double kDrivingI = 0;
//     public static final double kDrivingD = 0;
//     public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
//     public static final double kDrivingMinOutput = -1;
//     public static final double kDrivingMaxOutput = 1;

//     public static final double kTurningP = 1;
//     public static final double kTurningI = 0;
//     public static final double kTurningD = 0;
//     public static final double kTurningFF = 0;
//     public static final double kTurningMinOutput = -1;
//     public static final double kTurningMaxOutput = 1;

//     public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
//     public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

//     public static final int kDrivingMotorCurrentLimit = 50; // amps
//     public static final int kTurningMotorCurrentLimit = 20; // amps
//   }

//   public static final class OIConstants {
//     public static final int kDriverControllerPort = 0;
//     public static final double kDriveDeadband = 0.05;
//   }

//   public static final class AutoConstants {
//     public static final double kMaxSpeedMetersPerSecond = 3;
//     public static final double kMaxAccelerationMetersPerSecondSquared = 3;
//     public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
//     public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

//     public static final double kPXController = 1;
//     public static final double kPYController = 1;
//     public static final double kPThetaController = 1;

//     // Constraint for the motion profiled robot angle controller
//     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
//         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
//   }

//   public static final class NeoMotorConstants {
//     public static final double kFreeSpeedRpm = 5676;
//   }
// } */