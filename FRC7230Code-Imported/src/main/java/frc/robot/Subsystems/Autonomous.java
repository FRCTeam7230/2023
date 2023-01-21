package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private DriveSubsystem m_dDriveSubsystem;
  public Autonomous(){
    m_dDriveSubsystem = Mechanisms.driveSubsystem;

  }
  public void init(){
    m_dDriveSubsystem.resetEncoders();    
    autonomousTimer.reset();
    autonomousTimer.start();
  }
  public void execute(){

  }
  
}