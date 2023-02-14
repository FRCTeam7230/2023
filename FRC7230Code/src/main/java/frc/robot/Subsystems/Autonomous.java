package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private boolean search4Ball = true;
  private DriveSubsystem m_dDriveSubsystem;
  private CANSparkMax conveyor;
  private CANSparkMax shooter;
  private VictorSPX intake;
  private Solenoid intakeSol;

  public Autonomous(){
    m_dDriveSubsystem = Mechanisms.driveSubsystem;
    conveyor = Mechanisms.conveyorMotor;
    shooter = Mechanisms.shooterMotor;
    intake = Mechanisms.intakeMotor;
    intakeSol = Mechanisms.intakeSolenoid;

  }
  public void init(){
    // m_dDriveSubsystem.resetEncoders();    
    autonomousTimer.reset();
    autonomousTimer.start();
  }
  public void execute(){
    // 2 scenarious for autonomous period:
    // 1 - if robot is placed on the center grid, then during Auto it should score piece, then go backwards and engade chanrign station
    // 2 - if robot is place on the side grids, 


    // First phase - drive - drive to the hub and shoot ball
    // Second phase - drive backwards
    // Third phase - findNewBall - turn around, find ball
    // Forth phase - grabBall - drive toward the ball and grab it then

    // if (!m_stick.getRawButton(4)) { //emergency button Y

      //double ballAngleX = Mechanisms.vision.getAngleX();
      double ballAngleX = 0.0;

      // Phase 1 - Move forward to the hub
      
      if (autoState == "firstDrive") {
        System.out.println(autonomousTimer.get());
        if (autonomousTimer.get()<2.0){ 
          m_dDriveSubsystem.arcadeDrive(0,0);
          conveyor.set(0.5); 
          shooter.set(0.5);
        }
        else {
          autoState = "secondDrive";
          autonomousTimer.reset();
          autonomousTimer.start();
        }
      } 

      //Phase 2 - Drive backwards - do we need intake and conveyor running? To catch random balls
      else if (autoState == "secondDrive"){ 
        if (autonomousTimer.get() < 2.0){
          m_dDriveSubsystem.arcadeDrive(0, -0.7);
          // intakeSol.set(true);
          // intake.set(ControlMode.PercentOutput, 0.65);
          // conveyor.set(0.5);
          // shooter.set(-0.1);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          // intakeSol.set(false);
          // intake.set(ControlMode.PercentOutput, 0);
          // conveyor.set(0);
          // shooter.set(0);
          autoState = "findNewBall";
        }
      }

      // Phase 3 - Turn around and check where is the ball
      else if (autoState == "findNewBall") {

        // Phase 4 - Drive towards the ball and grab it
        if (search4Ball) { // Minuses - if the ball is swinging, if Robot  detected another colored object, not the ball
          
            intakeSol.set(true);
            intake.set(ControlMode.PercentOutput, 0.65);
            conveyor.set(0.5);
            // driveTrain.drive(true, driveModified);
            double speed = 0.9;
            double margin = 3;
            double angle = ballAngleX;

            if (angle>0 && angle>margin){ // Find right direction
              m_dDriveSubsystem.drive(speed/4, -speed/2);
            }
            else if (angle<0 && angle<-margin){
              m_dDriveSubsystem.drive(-speed/2, speed/4);
            }
            else if (Math.abs(angle) < margin){
              m_dDriveSubsystem.arcadeDrive(0, -speed/2);
            }
        }
        // Phase 5 - receive information from encoders - calculate the distance that the robot has traveled. 
        //Also, add code for "remembering" the angle of rotation of the robot - maybe encoder too.
        // Or, when moving to grab the ball, "remeber" the change in position (which direction they turned and how long) of left and right wheels - just turn in opposite direction
      }
  }
}