package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Mechanisms;
import edu.wpi.first.math.controller.*;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;


public class RunMechanisms {
    private static CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawRightSolenoid = Mechanisms.clawRightSolenoid, clawLeftSolenoid = Mechanisms.clawLeftSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    PIDController pid = new PIDController(kP,kI,kD);
    public static CANSparkMax PIDTest = new CANSparkMax(9, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder PIDEncoder = PIDTest.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    public static int mode1Encoder = 0; //high score
    public static int mode2Encoder = 0; //mid score
    public static int mode3Encoder = 0; //low score & pick up from ground
    public static int mode4Encoder = 0; //pick up from shelf
    public static int [] encoderModeVal = {mode1Encoder, mode2Encoder, mode3Encoder, mode4Encoder};
    //add value later
    static double encoderVal;
    public static RelativeEncoder armEncoder = armMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    public static int modeVal;
    public static double armError;




    public void PID(int button, int setPoint) {
      // Calculates the output of the PID algorithm based on the sensor reading
      // and sends it to a motor
      PIDTest.set(pid.calculate(PIDEncoder.getPosition(), setPoint));
    }


     
   public void extendArm(int button){


    }
   public void retractArm(CANSparkMax motor, int button){

     
   }
 


    public static void Arm(int mode) {
      //arm method should check which mode it is
      //update encoder value
      //rotate arm till value is met
      //stop rotate
      modeVal = encoderModeVal[mode - 1];
      encoderVal = armEncoder.getPosition();
      armError = modeVal - encoderVal;
      if (armError > 5) {
        //test "5" later
        armMotor.set(0.5);
      }
      else if (armError < -5) {
        armMotor.set(-0.5);
      }
      else {
        armMotor.set(0);
      }
  
    }
    //mode 1 Scoring High Cone Node
      
    /* 
      Encoder needs to measure
      arm then motor moves according to what angle is being measured
     */
   
  
  public void Claw(int button, Solenoid solenoid1, Solenoid solenoid2) {
    boolean previousstate = true;
    boolean state = m_stick.getRawButton(button);
    if (state && !previousstate) {
      solenoid1.set(state);
      solenoid2.set(state);
    }
    else {
      solenoid1.close();
      solenoid2.close();
    }
    
  }
}

  