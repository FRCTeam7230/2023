package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
public class RunMechanisms {
    private CANSparkMax shooterMotor = Mechanisms.shooterMotor, conveyorMotor = Mechanisms.conveyorMotor, climberMotor = Mechanisms.climberMotor; 
    private VictorSPX intakeMotor = Mechanisms.intakeMotor;
    private Solenoid intakeSolenoid = Mechanisms.intakeSolenoid, climberSolenoid = Mechanisms.climberSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    private boolean previousState = false;
    private boolean previousClimb = false;
    private boolean currentClimb;
    private Timer climberTimer = new Timer();
    private Timer shotTimer = new Timer();
    
   public void runCANMechanism(CANSparkMax motor, int button, double power, boolean invert, double offPower){
    boolean state = m_stick.getRawButton(button); 
    double newPower = power;
     if (invert){
       newPower*=-1;
     }
     if(state){
         motor.set(newPower);
     }
     else{
       motor.set(-offPower);
     }
     
   } 

   public void runSPXMechanism(VictorSPX motor, int button1, int button2, double power, boolean invert){
    double newPower = power;
    if (invert){
      newPower*=-1;
    }
    if(m_stick.getRawButton(button1) || m_stick.getRawButton(button2)){
        // DriverStation.reportWarning("running spxbutton "+button+"fwd",true);
        motor.set(ControlMode.PercentOutput,newPower);
    }
    else{
      motor.set(ControlMode.PercentOutput,0);
    }
  } 

   public void runPneumaticCompressor(Compressor comp, int button, boolean enabled){
    if(m_stick.getRawButton(button)){
        // DriverStation.reportWarning("running compressor",true);
        comp.enableDigital();

    }
    else{
      // DriverStation.reportWarning("compressor off",true);
      comp.disable();
    }
  } 
  public void runPneumaticSolenoid(Solenoid solenoid, int button, boolean enabled){
   if(m_stick.getRawButton(button) && enabled){
        // DriverStation.reportWarning("running solenoid",true);
        solenoid.set(true);

   }
   else if (enabled){
        // DriverStation.reportWarning("solenoid off",true);
        solenoid.set(false);
     
   }
 } 
// button1 = shoot, button2 = intake
  public void runShotAndIntake(int button1, int button2, int button3, double power, boolean enabled){
    boolean state1 = m_stick.getRawButton(button1);
    boolean state2 = m_stick.getRawButton(button2); 
    boolean state3 = m_stick.getRawButton(button3); 
      double newPower = power;
      //  if (state && !previousState){
      //    motor.set(-offPower);
      //  }
      if (previousState == false && state1 == true){
        shotTimer.reset();
        shotTimer.start();
      }
      previousState = state1;
      if(state1){
        // DriverStation.reportWarning("running button "+button1+"fwd",true);
        if (shotTimer.get()<0.0625){
          shooterMotor.set(0);
          conveyorMotor.set(-0.5);
        }
        else if (shotTimer.get()>=0.0625 &&  shotTimer.get()<0.5625){
          shooterMotor.set(newPower);
          conveyorMotor.set(0);
        }
        else if (shotTimer.get()>=0.5625){
          conveyorMotor.set(0.5);
          shooterMotor.set(newPower);
        }
        // conveyorMotor.set(0.5);
      }
      else{
        shooterMotor.set(-newPower/10);
      }
      if(state2 || state3){
          // DriverStation.reportWarning("running button "+button2+"fwd",true);
          conveyorMotor.set(0.5);
      }
      if (!state1 && !state2 && !state3){
        conveyorMotor.set(0);
      }
      runPneumaticSolenoid(intakeSolenoid, button2, enabled);
      runSPXMechanism(intakeMotor, button2, button3, 0.65, false);
  }

  /**
   * Move climber up and down based on button A and X
   * 
   * @author  Branden Tang
   */ 
  
  public void timerRestart(){
    climberTimer.reset();
    climberTimer.start();
  }
  public void runClimber(int BUTTON_X, int BUTTON_A) {
    currentClimb = m_stick.getRawButton(BUTTON_X);
    if (currentClimb && !previousClimb){
      timerRestart();
    }
    if (currentClimb) {//move climber up
      climberSolenoid.set(m_stick.getRawButton(BUTTON_X));
      // timerRestart();
      // DriverStation.reportWarning("RUN CLIMBER", true);
      if (climberTimer.get() < 0.1) {
        System.out.println("RUNNING CLIMB BACK");
        climberMotor.set(1);
        }
      else {
        System.out.println("RUNNING CLIMB ");
        climberMotor.set(-1);
        }
    }
    else {
      climberSolenoid.set(false);
      climberMotor.set(0);
    }

    if (m_stick.getRawButton(BUTTON_A)) {//move climber down
      // climberSolenoid.set(m_stick.getRawButton(BUTTON_A));
      // timerRestart();
      
      // if (climberTimer.get() < 0.1) {
      climberMotor.set(0.8);
      // }
    }
    else if (!m_stick.getRawButton(BUTTON_X)){
      // climberSolenoid.set(false);
      climberMotor.set(0);
    }
    previousClimb = currentClimb;
  }
}
