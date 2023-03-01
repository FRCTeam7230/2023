package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Limelight;
import frc.robot.Mechanisms;

public class Autonomous {
  private DriveSubsystem m_DriveSubsystem;
  private RunMechanisms m_RunMechanisms;
  private Timer autonomousTimer = new Timer();

  private String autoState;
  private boolean surpassedMargin = false;
  private boolean surpassedMargin2 = false;
  private double gyroAngle;
  private double gyroError;
  private double angleToTarget;
  public static boolean completedDrive = false;
  private boolean driveLeftMotor, driveRightMotor;

  
  public Autonomous(){
    m_DriveSubsystem = Mechanisms.driveSubsystem;
    m_RunMechanisms = Mechanisms.runMechanisms;
  }
  public void init(){   
    autoState = "first";
  }

  public void execute(boolean midPosition, boolean coneLoaded, boolean autonConePickup){
      // First phase - Score Piece High
      // Second phase - Drive forward
      // Third phase - Grab Piece
      // Fourth phase - level off robot
      // Fifth phase - Drive back, if middle autobalance
    
    if(autoState == "first") {
      if (coneLoaded){
        m_RunMechanisms.autonRotateArmToAngle(driveTrainConstants.coneHighAngleEncoderCounts);
      }
      else{
        m_RunMechanisms.autonRotateArmToAngle(driveTrainConstants.cubeHighAngleEncoderCounts);
      }
      if (m_RunMechanisms.autonCompletedRotating){
        autonomousTimer.reset();
        autonomousTimer.start();
        if (autonomousTimer.get()<= 0.3){ 
        m_RunMechanisms.autonToggleArmExtension(); 
        }
        if (autonomousTimer.get()<= 0.6 && autonomousTimer.get() > 0.3){ 
          m_RunMechanisms.toggleClaw(true);
        }
        else if (autonomousTimer.get()<= 0.9 && autonomousTimer.get() > 0.6){ 
            m_RunMechanisms.autonToggleArmExtension();
        }
        else{
          autoState = "second";
        }
      }
    }

    if(autoState == "second" && midPosition){
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.metersToPieceFromMiddle);
      if (completedDrive){
        autoState = "third";
      }
    }

    if (autoState == "second" && !midPosition){
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.metersToPieceFromSide);
      if (completedDrive){
        autoState = "third";
        m_DriveSubsystem.resetEncoders();
      }
    }
    if(autoState == "third"){
      
      angleToTarget = Limelight.getTargetAngleX(); 
      
      m_RunMechanisms.autonRotateArmToAngle(driveTrainConstants.lowPickupAngleEncoderCounts);
      if (m_RunMechanisms.autonCompletedRotating){
        if (angleToTarget>0 && angleToTarget>driveTrainConstants.smartAngleMarginVision){
          m_DriveSubsystem.drive(driveTrainConstants.smartSpeedVision/2, -driveTrainConstants.smartSpeedVision);
        }
        else if (angleToTarget<0 && angleToTarget<-driveTrainConstants.smartAngleMarginVision){
          m_DriveSubsystem.drive(-driveTrainConstants.smartSpeedVision, driveTrainConstants.smartSpeedVision/2);
        }
        else{
          autonomousTimer.reset();
          autonomousTimer.start();
          if (autonomousTimer.get()<= 0.3){ 
            m_RunMechanisms.autonToggleArmExtension(); 
          }
          if (autonomousTimer.get()<= 0.6 && autonomousTimer.get() > 0.3){ 
            m_RunMechanisms.toggleClaw(true);
          }
          else if (autonomousTimer.get()<= 0.9 && autonomousTimer.get() > 0.6){ 
            m_RunMechanisms.autonToggleArmExtension();
          }
          else{
            autoState = "fourth";
          }
        }
      } 
    }

    if(autoState == "fourth") {

      if (m_DriveSubsystem.getLeftDistance() < driveTrainConstants.driveMargin) {
        m_DriveSubsystem.drive(driveTrainConstants.slowSmartSpeed, 0);
      }
      else if (m_DriveSubsystem.getRightDistance() > - driveTrainConstants.driveMargin) {
        m_DriveSubsystem.drive(0, -driveTrainConstants.slowSmartSpeed);
      }
      else {
        m_DriveSubsystem.drive(0, 0);
        autoState = "fifth";
        DriveSubsystem.prevAuto = false;
      }
    }

    if(autoState == "fifth") {

      DriveTrain.invertAxis = 1;
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.metersToPieceFromSide);
      
      if(midPosition){
        if (completedDrive){
          if (Math.abs(gyroError)>driveTrainConstants.smartAngleMargin && !surpassedMargin){
            surpassedMargin = true;
          } 
          if (Math.abs(gyroError)>driveTrainConstants.smartAngleMargin2 && surpassedMargin && !surpassedMargin2){
            surpassedMargin2 = true;
          }
          gyroAngle = Mechanisms.gyro.getPitch();
          gyroError = driveTrainConstants.targetAngle - gyroAngle;
          if (gyroError > driveTrainConstants.smartAngleMargin || (!surpassedMargin && !surpassedMargin2 && gyroError>0.3)){
            m_DriveSubsystem.drive(driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed);
          }
          else if (gyroError>0.3 && gyroError > driveTrainConstants.smartAngleMargin && surpassedMargin){
            m_DriveSubsystem.drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
          }
          else if (gyroError < - driveTrainConstants.smartAngleMargin || (!surpassedMargin&& !surpassedMargin2  && gyroError<-0.3)){
            m_DriveSubsystem.drive(-driveTrainConstants.smartSpeed, -driveTrainConstants.smartSpeed);
          }
          else if (gyroError<-0.3 && gyroError <- driveTrainConstants.smartAngleMargin && surpassedMargin){
            m_DriveSubsystem.drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
          }
          else{
            m_DriveSubsystem.drive(0, 0);
          }
        }
      }
    }
  }
}