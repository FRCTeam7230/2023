package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.Encoder;

public class RunMechanisms {
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawRightSolenoid = Mechanisms.clawRightSolenoid, clawLeftSolenoid = Mechanisms.clawLeftSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    private final Encoder armMotorEncoder = new Encoder(7, 8, 9);

  private double encoderCounts;
  public boolean buttonPressed = false;
  private boolean prevButton = false;
  private Timer armTimer = new Timer();
  
  public boolean completedRotating = false;

  public void breakInArmMotor(){
    armTimer.reset();
    armTimer.start();
    if (armTimer.get() < 60){
      armMotor.set(0.5);
    }
    if (armTimer.get() > 60 && armTimer.get() <120){
      armMotor.set(-0.5);
    }

  }

  public void rotateArmToAngle(){
    if (m_stick.getRawButton(robotConstants.SHELF_PICKUP_BUTTON)){
      buttonPressed = true;
      encoderCounts = driveTrainConstants.shelfAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.HIGH_SCORE_BUTTON)) {
      buttonPressed = true;
      encoderCounts = driveTrainConstants.highAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.MID_SCORE_BUTTON)) {
      buttonPressed = true;
      encoderCounts = driveTrainConstants.midAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.LOW_PICKUP_BUTTON)) {
      buttonPressed = true;
      encoderCounts = driveTrainConstants.lowPickupAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.LOW_SCORE_BUTTON)){
      buttonPressed = true;
      encoderCounts = driveTrainConstants.lowScoreAngleEncoderCounts;
    }

    if (buttonPressed){
      if (!prevButton){
        armSolenoid.toggle();
      }
      // if (armMotorEncoder.get() < encoderCounts - driveTrainConstants.armAngleMargin) {
      //   armMotor.set(driveTrainConstants.armMotorSpeed);
      // }
      // else if (armMotorEncoder.get() > encoderCounts + driveTrainConstants.armAngleMargin){
      //   armMotor.set(-driveTrainConstants.armMotorSpeed);
      // }
      // else {
      //   armMotor.set(0);
      //   buttonPressed = false;
      //   //just in case
      // }
      Mechanisms.armPID.setReference(encoderCounts, ControlType.kPosition);
      if (encoderCounts == armMotorEncoder.get()){
        completedRotating = true;
      }
      else{
        completedRotating = false;
      }
    }   
    prevButton = buttonPressed;
  }

  public void autonRotateArmToAngle(double autonEncoderCounts){
    // stillRotating = true;
    // completedRotating = false;
    // while (stillRotating){
    //   if (armMotorEncoder.get() < autonEncoderCounts - driveTrainConstants.armAngleMargin) {
    //     armMotor.set(driveTrainConstants.armMotorSpeed);
    //   }
    //   else if (armMotorEncoder.get() > autonEncoderCounts + driveTrainConstants.armAngleMargin){
    //     armMotor.set(-driveTrainConstants.armMotorSpeed);
    //   }
    //   else{
    //     armMotor.set(0);
    //     stillRotating = false;
    //     completedRotating = true;
    //   }
    // }
    
    Mechanisms.armPID.setReference(autonEncoderCounts, ControlType.kPosition);
  }

   public void autonToggleArmExtension(){
        armSolenoid.toggle();
   } 
   public void toggleClaw(boolean auto){
    if (m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON) || auto){
        clawRightSolenoid.toggle();
        clawLeftSolenoid.toggle();
    }
  } 
}
