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

    
    
    if(autoState == "first") {
      // First phase - 
      // Second phase - 
      // Third phase - 
      // Fourth phase - 
    }
      
    
  }
}