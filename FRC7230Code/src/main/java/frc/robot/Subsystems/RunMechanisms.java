package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;

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
  private boolean buttonPressed = false;
  private boolean stillRotating;

  public void rotateArmToAngle(){
    if (m_stick.getRawButton(robotConstants.SHELF_ANGLE_BUTTON)){
      buttonPressed = true;
      encoderCounts = driveTrainConstants.shelfAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.HIGH_ANGLE_BUTTON)) {
      buttonPressed = true;
      encoderCounts = driveTrainConstants.highAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.MID_ANGLE_BUTTON)) {
      buttonPressed = true;
      encoderCounts = driveTrainConstants.midAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.LOW_ANGLE_BUTTON)) {
      buttonPressed = true;
      encoderCounts = driveTrainConstants.lowAngleEncoderCounts;
    }

    if (buttonPressed){
      if (armMotorEncoder.get() < encoderCounts - driveTrainConstants.armAngleMargin) {
        armMotor.set(driveTrainConstants.armMotorSpeed);
      }
      else if (armMotorEncoder.get() > encoderCounts + driveTrainConstants.armAngleMargin){
        armMotor.set(-driveTrainConstants.armMotorSpeed);
      }
      else {
        armMotor.set(0);
        buttonPressed = false;
        //just in case
      }
    }   
  }
  public void autonRotateArmToAngle(double autonEncoderCounts){
    stillRotating = true;
    while (stillRotating){
      if (armMotorEncoder.get() < autonEncoderCounts - driveTrainConstants.armAngleMargin) {
        armMotor.set(driveTrainConstants.armMotorSpeed);
      }
      else if (armMotorEncoder.get() > autonEncoderCounts + driveTrainConstants.armAngleMargin){
        armMotor.set(-driveTrainConstants.armMotorSpeed);
      }
      else{
        armMotor.set(0);
        stillRotating = false;
      }
    }
  }

   public void toggleArmExtension(boolean auto){
    if (m_stick.getRawButtonPressed(robotConstants.ARM_EXTENSION_TOGGLE_BUTTON) || auto){
        armSolenoid.toggle();
    }
   } 
   public void toggleClaw(boolean auto){
    if (m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON) || auto){
        clawRightSolenoid.toggle();
        clawLeftSolenoid.toggle();
    }
  } 
}
