package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

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

  public void rotateArmToAngle(int mode){ // mode from 1 to 5, scoring: 1-high, 2-mid, 3-low; pick up: 4-ground, 5-shelf
    if (mode == 5) { // Pick up from Shelf
      buttonPressed = true;
      encoderCounts = driveTrainConstants.shelfAngleEncoderCounts;
    }
    else if (mode == 1) { // Scoring High
      buttonPressed = true;
      encoderCounts = driveTrainConstants.highAngleEncoderCounts;
    }
    else if (mode == 2) { // Scoring Mid
      buttonPressed = true;
      encoderCounts = driveTrainConstants.midAngleEncoderCounts;
    }
    else if (mode == 4) { // Pick up from Ground
      buttonPressed = true;
      encoderCounts = driveTrainConstants.lowPickupAngleEncoderCounts;
    }
    else { // Scoring Low
      buttonPressed = true;
      encoderCounts = driveTrainConstants.lowScoreAngleEncoderCounts;
    }

    if (buttonPressed){
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
    }   
  }
  public void autonRotateArmToAngle(double autonEncoderCounts){
    // stillRotating = true;
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
    //   }
    // }
    
    Mechanisms.armPID.setReference(autonEncoderCounts, ControlType.kPosition);
  }

   public  void toggleArmExtension(boolean auto){ // auto mean autonomous
    if (m_stick.getRawButtonPressed(robotConstants.ARM_EXTENSION_TOGGLE_BUTTON) || auto){
        armSolenoid.toggle();
    }
   } 
   public void toggleClaw(boolean auto){ // auto mean autonomous
    if (m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON) || auto){
        clawRightSolenoid.toggle();
        clawLeftSolenoid.toggle();
    }
  } 
}
