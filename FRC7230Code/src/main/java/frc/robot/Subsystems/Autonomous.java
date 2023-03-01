package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private DriveSubsystem m_DriveSubsystem;
  private CANSparkMax a_ArmMotor; 
  private Solenoid a_ArmSolenoid, a_ClawRightSolenoid, a_ClawLeftSolenoid;
  public Autonomous(){
    m_DriveSubsystem = Mechanisms.driveSubsystem;
    a_ArmMotor = Mechanisms.armMotor; 
    a_ArmSolenoid = Mechanisms.armSolenoid;
    a_ClawRightSolenoid = Mechanisms.clawRightSolenoid;
    a_ClawLeftSolenoid = Mechanisms.clawLeftSolenoid;
  }
  public void init(){
    // m_DriveSubsystem.resetEncoders();    
    autonomousTimer.reset();
    autonomousTimer.start();
    autoState = "firstDrive";
  }
  public void execute(boolean midPosition){

    
      // First phase - Score Piece High
      // Second phase - Drive forward
      // Third phase - Grab Piece
      // Fourth phase - Drive back, if middle autobalance
      
      if (autoState == "firstDrive") {
      // System.out.println(autonomousTimer.get());
      //   if (autonomousTimer.get()<2.0){ 
          
      //   }
      //   else {
      //     autoState = "secondDrive";
      //     autonomousTimer.reset();
      //     autonomousTimer.start();
      //   }
      // } 
    
    if(autoState == "first") {
      // First phase - 
      // Second phase - 
      // Third phase - 
      // Fourth phase - 
      RunMechanisms.rotateArmToAngle(0);
      //Mechanisms.armMotor.set(1.0);
      if(DriveTrain.armMotorEncoder.get() == driveTrainConstants.highAngleEncoderCounts){
        Mechanisms.armMotor.set(0);
      }
      RunMechanisms.toggleArmExtension(0);
      RunMechanisms.toggleClaw();
      RunMechanisms.toggleArmExtension(0);
      Mechanisms.armSolenoid.set(true);
      Mechanisms.clawLeftSolenoid.set(true);
      Mechanisms.clawRightSolenoid.set(true);
      Mechanisms.armSolenoid.set(false);
    }
      
    
  }
}}