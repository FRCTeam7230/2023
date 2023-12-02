/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;

public class DriveSubsystem extends SubsystemBase {
  
  public boolean completedDrive;
  public double initialLeft, initialRight;
  public CANSparkMax l_motor1 = new CANSparkMax(robotConstants.L1MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax r_motor1 = new CANSparkMax(robotConstants.R1MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax l_motor2 = new CANSparkMax(robotConstants.L2MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax r_motor2 = new CANSparkMax(robotConstants.R2MOTOR_ID, CANSparkMax.MotorType.kBrushless);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(l_motor1, l_motor2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(r_motor1, r_motor2);
  public final SparkMaxPIDController lController = l_motor2.getPIDController();
  public final SparkMaxPIDController rController = r_motor1.getPIDController();

  public static boolean prevAuto = false;
  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  public final RelativeEncoder m_leftEncoder = 
  l_motor2.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  // The right-side drive encoder
  public final RelativeEncoder m_rightEncoder = 
  r_motor1.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    m_leftEncoder.setPositionConversionFactor(driveTrainConstants.kEncoderMetersPerMotorRev);
    m_rightEncoder.setPositionConversionFactor(driveTrainConstants.kEncoderMetersPerMotorRev);

    initialLeft = m_leftEncoder.getPosition();
    initialRight = m_rightEncoder.getPosition();
    
    m_drive.setSafetyEnabled(false);

    // lController.setP(driveTrainConstants.kP);
    // lController.setI(driveTrainConstants.kI);
    // lController.setD(driveTrainConstants.kD);
    // lController.setFF(driveTrainConstants.kFF);
    // rController.setP(driveTrainConstants.kP);
    // rController.setI(driveTrainConstants.kI);
    // rController.setD(driveTrainConstants.kD);
    // rController.setFF(driveTrainConstants.kFF);
  }


  

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void drive(double left, double right) {
    m_drive.tankDrive(left, -right);
  }
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    initialLeft = m_leftEncoder.getPosition();
    initialRight = m_rightEncoder.getPosition();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((m_leftEncoder.getPosition()-initialLeft) - (m_rightEncoder.getPosition()-initialRight)) / 2.0;
  }

  public double getLeftDistance(){
    return m_leftEncoder.getPosition()-initialLeft;
  }
  public double getRightDistance(){
    return m_rightEncoder.getPosition()-initialRight;
  }
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public double getError(){
    return m_leftEncoder.getPosition()-m_rightEncoder.getPosition();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  public void resetAuton(){
    if  (!prevAuto) {
      resetEncoders();
      Autonomous.completedDrive = false;
    }
  }
  public void autonDriveSetDistance(double distance){ 
    
    prevAuto = true;
    // System.out.println(getAverageEncoderDistance());
    // System.out.println(distance);
      // distance may be positive and negative, so we need to take the absolute values 
    if (getAverageEncoderDistance() < distance - driveTrainConstants.driveMargin) {
      drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
    }
    else if (getAverageEncoderDistance() > distance + driveTrainConstants.driveMargin) {
      drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
    }
    else {
      drive(0,0);
      Autonomous.completedDrive = true;
    }
  } 
}
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.util.WPIUtilJNI;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import frc.robot.Constants.DriveConstants;
// import frc.utils.SwerveUtils;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class DriveSubsystem extends SubsystemBase {
//   // Create MAXSwerveModules
//   private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
//       DriveConstants.kFrontLeftDrivingCanId,
//       DriveConstants.kFrontLeftTurningCanId,
//       DriveConstants.kFrontLeftChassisAngularOffset);

//   private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
//       DriveConstants.kFrontRightDrivingCanId,
//       DriveConstants.kFrontRightTurningCanId,
//       DriveConstants.kFrontRightChassisAngularOffset);

//   private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
//       DriveConstants.kRearLeftDrivingCanId,
//       DriveConstants.kRearLeftTurningCanId,
//       DriveConstants.kBackLeftChassisAngularOffset);

//   private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
//       DriveConstants.kRearRightDrivingCanId,
//       DriveConstants.kRearRightTurningCanId,
//       DriveConstants.kBackRightChassisAngularOffset);

//   // The gyro sensor
//   private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

//   // Slew rate filter variables for controlling lateral acceleration
//   private double m_currentRotation = 0.0;
//   private double m_currentTranslationDir = 0.0;
//   private double m_currentTranslationMag = 0.0;

//   private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
//   private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
//   private double m_prevTime = WPIUtilJNI.now() * 1e-6;

//   // Odometry class for tracking robot pose
//   SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
//       DriveConstants.kDriveKinematics,
//       Rotation2d.fromDegrees(m_gyro.getAngle()),
//       new SwerveModulePosition[] {
//           m_frontLeft.getPosition(),
//           m_frontRight.getPosition(),
//           m_rearLeft.getPosition(),
//           m_rearRight.getPosition()
//       });

//   /** Creates a new DriveSubsystem. */
//   public DriveSubsystem() {
//   }

//   @Override
//   public void periodic() {
//     // Update the odometry in the periodic block
//     m_odometry.update(
//         Rotation2d.fromDegrees(m_gyro.getAngle()),
//         new SwerveModulePosition[] {
//             m_frontLeft.getPosition(),
//             m_frontRight.getPosition(),
//             m_rearLeft.getPosition(),
//             m_rearRight.getPosition()
//         });
//   }

//   /**
//    * Returns the currently-estimated pose of the robot.
//    *
//    * @return The pose.
//    */
//   public Pose2d getPose() {
//     return m_odometry.getPoseMeters();
//   }

//   /**
//    * Resets the odometry to the specified pose.
//    *
//    * @param pose The pose to which to set the odometry.
//    */
//   public void resetOdometry(Pose2d pose) {
//     m_odometry.resetPosition(
//         Rotation2d.fromDegrees(m_gyro.getAngle()),
//         new SwerveModulePosition[] {
//             m_frontLeft.getPosition(),
//             m_frontRight.getPosition(),
//             m_rearLeft.getPosition(),
//             m_rearRight.getPosition()
//         },
//         pose);
//   }

//   /**
//    * Method to drive the robot using joystick info.
//    *
//    * @param xSpeed        Speed of the robot in the x direction (forward).
//    * @param ySpeed        Speed of the robot in the y direction (sideways).
//    * @param rot           Angular rate of the robot.
//    * @param fieldRelative Whether the provided x and y speeds are relative to the
//    *                      field.
//    * @param rateLimit     Whether to enable rate limiting for smoother control.
//    */
//   public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
//     double xSpeedCommanded;
//     double ySpeedCommanded;

//     if (rateLimit) {
//       // Convert XY to polar for rate limiting
//       double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
//       double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

//       // Calculate the direction slew rate based on an estimate of the lateral acceleration
//       double directionSlewRate;
//       if (m_currentTranslationMag != 0.0) {
//         directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
//       } else {
//         directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
//       }
      

//       double currentTime = WPIUtilJNI.now() * 1e-6;
//       double elapsedTime = currentTime - m_prevTime;
//       double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
//       if (angleDif < 0.45*Math.PI) {
//         m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
//         m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
//       }
//       else if (angleDif > 0.85*Math.PI) {
//         if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
//           // keep currentTranslationDir unchanged
//           m_currentTranslationMag = m_magLimiter.calculate(0.0);
//         }
//         else {
//           m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
//           m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
//         }
//       }
//       else {
//         m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
//         m_currentTranslationMag = m_magLimiter.calculate(0.0);
//       }
//       m_prevTime = currentTime;
      
//       xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
//       ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
//       m_currentRotation = m_rotLimiter.calculate(rot);


//     } else {
//       xSpeedCommanded = xSpeed;
//       ySpeedCommanded = ySpeed;
//       m_currentRotation = rot;
//     }

//     // Convert the commanded speeds into the correct units for the drivetrain
//     double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
//     double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
//     double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

//     var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
//         fieldRelative
//             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
//             : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
//     SwerveDriveKinematics.desaturateWheelSpeeds(
//         swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
//     m_frontLeft.setDesiredState(swerveModuleStates[0]);
//     m_frontRight.setDesiredState(swerveModuleStates[1]);
//     m_rearLeft.setDesiredState(swerveModuleStates[2]);
//     m_rearRight.setDesiredState(swerveModuleStates[3]);
//   }

//   /**
//    * Sets the wheels into an X formation to prevent movement.
//    */
//   public void setX() {
//     m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
//     m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
//     m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
//     m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
//   }

//   /**
//    * Sets the swerve ModuleStates.
//    *
//    * @param desiredStates The desired SwerveModule states.
//    */
//   public void setModuleStates(SwerveModuleState[] desiredStates) {
//     SwerveDriveKinematics.desaturateWheelSpeeds(
//         desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
//     m_frontLeft.setDesiredState(desiredStates[0]);
//     m_frontRight.setDesiredState(desiredStates[1]);
//     m_rearLeft.setDesiredState(desiredStates[2]);
//     m_rearRight.setDesiredState(desiredStates[3]);
//   }

//   /** Resets the drive encoders to currently read a position of 0. */
//   public void resetEncoders() {
//     m_frontLeft.resetEncoders();
//     m_rearLeft.resetEncoders();
//     m_frontRight.resetEncoders();
//     m_rearRight.resetEncoders();
//   }

//   /** Zeroes the heading of the robot. */
//   public void zeroHeading() {
//     m_gyro.reset();
//   }

//   /**
//    * Returns the heading of the robot.
//    *
//    * @return the robot's heading in degrees, from -180 to 180
//    */
//   public double getHeading() {
//     return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
//   }

//   /**
//    * Returns the turn rate of the robot.
//    *
//    * @return The turn rate of the robot, in degrees per second
//    */
//   public double getTurnRate() {
//     return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
//   }
// }