/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.driveTrainConstants;
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

    lController.setP(driveTrainConstants.kP);
    lController.setI(driveTrainConstants.kI);
    lController.setD(driveTrainConstants.kD);
    lController.setFF(driveTrainConstants.kFF);
    rController.setP(driveTrainConstants.kP);
    rController.setI(driveTrainConstants.kI);
    rController.setD(driveTrainConstants.kD);
    rController.setFF(driveTrainConstants.kFF);
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
    return (m_leftEncoder.getPosition()-initialLeft + m_rightEncoder.getPosition()-initialRight) / 2.0;
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

  public void autonDriveSetDistance(double distance){
    resetEncoders();
    completedDrive = false;
    if (getAverageEncoderDistance() < distance - driveTrainConstants.driveMargin) {
      drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
    } 
    else if (getAverageEncoderDistance() > distance + driveTrainConstants.driveMargin) {
      drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
    }
    else {
      drive(0,0);
      completedDrive = true;
    }
  // System.out.println("LEFT: " + m_robotDrive.getLeftDistance());
  // System.out.println("RIGHT: " + m_robotDrive.getRightDistance());
  // m_robotDrive.lController.setReference(m_robotDrive.m_leftEncoder.getPosition()+ driveTrainConstants.midScoreMeters * driveTrainConstants.kEncoderMetersPerMotorRev, CANSparkMax.ControlType.kPosition);
  // m_robotDrive.rController.setReference(m_robotDrive.m_rightEncoder.getPosition()+driveTrainConstants.midScoreMeters * driveTrainConstants.kEncoderMetersPerMotorRev, CANSparkMax.ControlType.kPosition);
}   
  

}