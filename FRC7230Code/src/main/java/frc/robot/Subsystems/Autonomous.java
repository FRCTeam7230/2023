package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private boolean search4Ball = true;
  private DriveSubsystem m_dDriveSubsystem;
  private CANSparkMax conveyor;
  private CANSparkMax shooter;
  private VictorSPX intake;
  private double error;
  private Solenoid intakeSol;
  private final double k=0.8;
  private boolean surpassedMargin = true;
  private boolean surpassedMargin2 = true;
  public Autonomous(){
    m_dDriveSubsystem = Mechanisms.driveSubsystem;
    conveyor = Mechanisms.conveyorMotor;
    shooter = Mechanisms.shooterMotor;
    intake = Mechanisms.intakeMotor;
    intakeSol = Mechanisms.intakeSolenoid;

  }
  public void init(){
    m_dDriveSubsystem.resetEncoders();    
    autonomousTimer.reset();
    autonomousTimer.start();
    autoState = "firstDrive";
  }
  public void execute(){

      // Phase 1 - Move forward to the hub
      error = driveTrainConstants.targetAngle-Mechanisms.gyro.getPitch();
      if (autoState == "firstDrive") {
        System.out.println(autonomousTimer.get());
        if (Math.abs(error) < driveTrainConstants.smartAngleMargin){ 
          m_dDriveSubsystem.drive(0.6,0.6);
          conveyor.set(0.5); 
          shooter.set(0.5);
        }
        else {
          autoState = "tilting";
          autonomousTimer.reset();
          autonomousTimer.start();
        }
      } 

      //Phase 2 - Drive backwards - do we need intake and conveyor running? To catch random balls
      else if (autoState == "tilting"){ 
        if (autonomousTimer.get() < 1.5){
          m_dDriveSubsystem.drive(0.55, 0.55);
          // intakeSol.set(true);
          // intake.set(ControlMode.PercentOutput, 0.65);
          // conveyor.set(0.5);
          // shooter.set(-0.1);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          autoState = "smart";
        }
      }

      // Phase 3 - Turn around and check where is the ball
      else if (autoState == "smart") {
        
        if (Math.abs(error)>driveTrainConstants.smartAngleMargin && !surpassedMargin){
          surpassedMargin = true;
          DriverStation.reportWarning("MARGIN PASSED", false);
      }
      if (Math.abs(error)>driveTrainConstants.smartAngleMargin2 && surpassedMargin && !surpassedMargin2){
          surpassedMargin2 = true;
          DriverStation.reportWarning("MARGIN 2 PASSED", false);
      }
        System.out.println(error);
      if (error > driveTrainConstants.smartAngleMargin || (!surpassedMargin && !surpassedMargin2 && error>0.3)){
        m_dDriveSubsystem.drive(driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed);
          SmartDashboard.putString("tilted forward", " driving back");
          System.out.println("forward");
      }
      else if (error>0.3 && error > driveTrainConstants.smartAngleMargin2 && surpassedMargin){
        m_dDriveSubsystem.drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
          SmartDashboard.putString("tilted forward", " driving back");
          System.out.println("forward slow");
      }
      else if (error < - driveTrainConstants.smartAngleMargin || (!surpassedMargin && !surpassedMargin2 && error<-0.3)){
        m_dDriveSubsystem.drive(-driveTrainConstants.smartSpeed, -driveTrainConstants.smartSpeed);
          System.out.println("backward");
          SmartDashboard.putString("tilted back", " driving forward");
      }
      else if (error<-0.3 && error < -driveTrainConstants.smartAngleMargin2 && surpassedMargin){
        m_dDriveSubsystem.drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
          SmartDashboard.putString("tilted forward", " driving back");
          System.out.println("backward slow");
      }
      else{
        m_dDriveSubsystem.drive(0, 0);
        System.out.println("Balanced");
          SmartDashboard.putString("balanced", "stopping");
      }
      }
  }
}