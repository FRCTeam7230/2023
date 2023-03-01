package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Mechanisms;
import edu.wpi.first.math.controller.*;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

public class RunMechanisms {
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawRightSolenoid = Mechanisms.clawRightSolenoid, clawLeftSolenoid = Mechanisms.clawLeftSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    PIDController pid = new PIDController(kP,kI,kD);
    public static CANSparkMax PIDTest = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    public final static RelativeEncoder PIDEncoder = PIDTest.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    public void PID(int button, int setPoint) {
      // Calculates the output of the PID algorithm based on the sensor reading
      // and sends it to a motor
      PIDTest.set(pid.calculate(PIDEncoder.getPosition(), setPoint));
    }
    public static void toggleClaw(){
      //insert claw code
    }
    

   
   public static void toggleArmExtension(int button){
    
     
   } 
   public static void rotateArmToAngle(int button){
    
    
   }

   
   public void extendArm(int button){
    
     
   } 
   public void retractArm(CANSparkMax motor, int button){
    
     
   }
}
