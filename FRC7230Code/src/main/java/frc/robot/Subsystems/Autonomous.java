package frc.robot.Subsystems;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
import frc.robot.Constants.*;

public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private DriveSubsystem m_DriveSubsystem;
  private RunMechanisms m_RunMechanisms;
  
  
  public Autonomous(){
    m_DriveSubsystem = Mechanisms.driveSubsystem;
    m_RunMechanisms = Mechanisms.runMechanisms;
  }
  public void init(){   
    autonomousTimer.reset();
    autonomousTimer.start();
    autoState = "first";
  }
  public void execute(boolean midPosition){
      // First phase - Score Piece High
      // Second phase - Drive forward
      // Third phase - Grab Piece
      // Fourth phase - Drive back, if middle autobalance
      
    
    
    if(autoState == "first") {
      m_RunMechanisms.autonRotateArmToAngle(driveTrainConstants.highAngleEncoderCounts);
      m_RunMechanisms.toggleArmExtension(true);
      m_RunMechanisms.toggleClaw(true);
      m_RunMechanisms.toggleArmExtension(true);
      //timer needed?
      autoState = "second";
    }
    if(autoState == "second" && midPosition){
      m_DriveSubsystem.autonDriveSetDistance(-driveTrainConstants.distanceToPieceFromMiddle);
      if (m_DriveSubsystem.completedDrive){
        autoState = "third";
      }
    }
    if (autoState == "second" && !midPosition){
      m_DriveSubsystem.autonDriveSetDistance(-driveTrainConstants.distanceToPieceFromSide);
      if (m_DriveSubsystem.completedDrive){
        autoState = "third";
      }
    }
    if(autoState == "third"){
      m_RunMechanisms.autonRotateArmToAngle(driveTrainConstants.lowAngleEncoderCounts);
      m_RunMechanisms.toggleArmExtension(true);
      m_RunMechanisms.toggleClaw(true);
      m_RunMechanisms.toggleArmExtension(true);
    }
    if(autoState == "fourth" && midPosition){
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.distanceToPieceFromSide);
    }
    if(autoState == "fourth" && midPosition){
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.distanceToPieceFromMiddle);
      if (m_DriveSubsystem.completedDrive){
         //autobalance
      }
    }
  }
}