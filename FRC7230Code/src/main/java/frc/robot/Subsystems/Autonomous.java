package frc.robot.Subsystems;


import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
import frc.robot.Constants.*;

public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private DriveSubsystem m_DriveSubsystem;
  private RunMechanisms m_RunMechanisms;
  private boolean surpassedMargin = false;
  private boolean surpassedMargin2 = false;
  private double gyroAngle;
  private double error;
  boolean armExtendComplete = true;
  
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
      if(m_RunMechanisms.rotateComplete){
        autonomousTimer.reset();
        autonomousTimer.start();
        if (autonomousTimer.get() < 0.3) {
          m_RunMechanisms.toggleArmExtension(true);
        }
        if(autonomousTimer.get() > 0.3 && autonomousTimer.get() < 0.6) {
          m_RunMechanisms.toggleClaw(true);
        }
        if(autonomousTimer.get() < 0.6 && autonomousTimer.get() < 0.9) {
          m_RunMechanisms.toggleArmExtension(true);
        }
        if(autonomousTimer.get() > 0.9) {
          autoState = "second";
          autonomousTimer.reset();
          autonomousTimer.start();
        }
      }
      //timer needed? for 0.3sec-for now
      //autoState = "second";
    }
    if(autoState == "second" && midPosition){
      m_DriveSubsystem.autonDriveSetDistance(-driveTrainConstants.metersToPieceFromMiddle);
      if (m_DriveSubsystem.completedDrive){
        autoState = "third";
      }
    }
    if (autoState == "second" && !midPosition){
      m_DriveSubsystem.autonDriveSetDistance(-driveTrainConstants.metersToPieceFromSide);
      //may need to turn robot towards piece before starting pickup
      if (m_DriveSubsystem.completedDrive){
        autoState = "third";
      }
    }
    if(autoState == "third"){
      m_RunMechanisms.autonRotateArmToAngle(driveTrainConstants.lowPickupAngleEncoderCounts);
      m_RunMechanisms.toggleArmExtension(true);
      m_RunMechanisms.toggleClaw(true);
      m_RunMechanisms.toggleArmExtension(true);
    }
    if(autoState == "fourth" && !midPosition){
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.metersToPieceFromSide);
    }
    if(autoState == "fourth" && midPosition){
      m_DriveSubsystem.autonDriveSetDistance(driveTrainConstants.metersFromPieceToBalance);
      if (m_DriveSubsystem.completedDrive){
            if (Math.abs(error)>driveTrainConstants.smartAngleMargin && !surpassedMargin){
                surpassedMargin = true;
            }
            if (Math.abs(error)>driveTrainConstants.smartAngleMargin2 && surpassedMargin && !surpassedMargin2){
                surpassedMargin2 = true;
            }
            gyroAngle = Mechanisms.gyro.getPitch();
            error = driveTrainConstants.targetAngle - gyroAngle;
            System.out.println(error);
            
            if (error > driveTrainConstants.smartAngleMargin || (!surpassedMargin && !surpassedMargin2 && error>0.3)){
                m_DriveSubsystem.drive(driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed);
                System.out.println("forward");
            }
            else if (error>0.3 && error > driveTrainConstants.smartAngleMargin && surpassedMargin){
                m_DriveSubsystem.drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
                System.out.println("forward");
            }
            else if (error < - driveTrainConstants.smartAngleMargin || (!surpassedMargin&& !surpassedMargin2  && error<-0.3)){
                m_DriveSubsystem.drive(-driveTrainConstants.smartSpeed, -driveTrainConstants.smartSpeed);
                System.out.println("backward");
            }
            else if (error<-0.3 && error <- driveTrainConstants.smartAngleMargin && surpassedMargin){
                m_DriveSubsystem.drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
                System.out.println("forward");
            }
            else{
                m_DriveSubsystem.drive(0, 0);
            }
      }
    }
  }
}