package frc.robot.Subsystems;

import frc.robot.Limelight;
import frc.robot.Mechanisms;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class RunMechanisms {
    public boolean rotateComplete;
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawSolenoid = Mechanisms.clawSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    private final SparkMaxAbsoluteEncoder armMotorEncoder = Mechanisms.armMotorEncoder;
    private final PIDController armController = Mechanisms.armController;
    
  private double neededEncoderCounts;
  public boolean buttonPressed = false;
  private boolean prevButton = false;
  private boolean needExtend;
  public boolean autonCompletedRotating = false;
  public String armAngle = "Ground Pickup";
  public boolean completedRotating = false;

  public double getEncoderPosition(){
    double pos = armMotorEncoder.getPosition();
    if (pos > 200){
      return armMotorEncoder.getPosition()-360;
    }
    else {
      return armMotorEncoder.getPosition();
    }
  }

  public void toggleArm(){
    if ((getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition() < driveTrainConstants.armLowerExtension) || (getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
      armSolenoid.toggle();
    }
    else{
      armSolenoid.set(false);
    }
  }
  // overloaded
  public void toggleArm(boolean value){
    if ((getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition() < driveTrainConstants.armLowerExtension) || (getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
      armSolenoid.set(value);
    }
    else{
      armSolenoid.set(false);
    }
  }
  public void checkArm(){
    if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition() < driveTrainConstants.armLowerExtension) && !(getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
      toggleArm(false);
    }

  }
  public void rotateArmToAngle(){
    if (m_stick.getRawButton(robotConstants.SHELF_PICKUP_BUTTON)){
      buttonPressed = true;
      needExtend = true;
      armAngle = "Shelf";
      neededEncoderCounts = driveTrainConstants.shelfAngleEncoderCounts;
    }
    else if (m_stick.getRawButton(robotConstants.HIGH_SCORE_BUTTON)) {
      buttonPressed = true;
      needExtend = true;
      armAngle = "High Scoring";
      if (Limelight.coneTarget){
        neededEncoderCounts = driveTrainConstants.coneHighAngleEncoderCounts;
     }
     else{
       neededEncoderCounts = driveTrainConstants.cubeHighAngleEncoderCounts;
     }
    }
    else if (m_stick.getRawButton(robotConstants.MID_SCORE_BUTTON)) {
      buttonPressed = true;
      needExtend = false;
      armAngle = "Middle Scoring";
      if (Limelight.coneTarget){
        neededEncoderCounts = driveTrainConstants.coneMidAngleEncoderCounts;
     }
     else{
       neededEncoderCounts = driveTrainConstants.cubeMidAngleEncoderCounts;
     }
    }
    else if (m_stick.getRawButton(robotConstants.GROUND_PICKUP_BUTTON)) {
      buttonPressed = true;
      needExtend = true;
      armAngle = "Ground Pickup";
      neededEncoderCounts = driveTrainConstants.lowPickupAngleEncoderCounts;
    }
    else if (m_stick.getRawButton(robotConstants.LOW_SCORE_BUTTON)){
      buttonPressed = true;
      needExtend = false;
      armAngle = "Low Scoring";
      neededEncoderCounts = driveTrainConstants.lowScoreAngleEncoderCounts;
    }
    else{
      buttonPressed = false;
    }

    if (buttonPressed){
      if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit+driveTrainConstants.armAngleMargin && getEncoderPosition()<driveTrainConstants.armUpperLimit-driveTrainConstants.armAngleMargin)){
        armMotor.set(0);
      }
      else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
        armMotor.set(driveTrainConstants.armMotorSpeed);
      }
      else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
        armMotor.set(-driveTrainConstants.armMotorSpeed);
      }
      else {
        armMotor.set(0);
        buttonPressed = false;
      }
      // armController.setSetpoint(neededEncoderCounts);
      // armMotor.set(armController.calculate(getEncoderPosition(),neededEncoderCounts));
      // if (armMotorEncoder.get() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
      //   armMotor.set(driveTrainConstants.armMotorSpeed);
      // }
      // else if (armMotorEncoder.get() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
      //   armMotor.set(-driveTrainConstants.armMotorSpeed);
      // }
      // else {
      //   armMotor.set(0);
      //   buttonPressed = false;
      // }
      // armMotor.set(armController.calculate(armMotorEncoder.getPosition(), neededEncoderCounts));
      if (neededEncoderCounts == armMotorEncoder.getPosition()){
        completedRotating = true;
        if (!prevButton && needExtend && neededEncoderCounts != driveTrainConstants.lowPickupAngleEncoderCounts){
          toggleArm();
        }
      }
      else if (neededEncoderCounts == driveTrainConstants.lowPickupAngleEncoderCounts && armMotorEncoder.getPosition() == driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.groundPickupAngleMargin){
        toggleArm(true);
      }
      else{
        completedRotating = false;
        if (armMotorEncoder.getPosition() > driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.groundPickupAngleMargin){
          toggleArm(false);
        }
      }
    }   
    prevButton = buttonPressed;
  }

  public void autonRotateArmToAngle(double autonEncoderCounts){
    autonCompletedRotating = false;
    //   if (armMotorEncoder.get() < autonEncoderCounts - driveTrainConstants.armAngleMargin) {
    //     armMotor.set(driveTrainConstants.armMotorSpeed);
    //   }
    //   else if (armMotorEncoder.get() > autonEncoderCounts + driveTrainConstants.armAngleMargin){
    //     armMotor.set(-driveTrainConstants.armMotorSpeed);
    //   }
    //   else{
    //     armMotor.set(0);
    //     completedRotating = true;
    //   }
    // }
    
    if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit+driveTrainConstants.armAngleMargin && getEncoderPosition()<driveTrainConstants.armUpperLimit-driveTrainConstants.armAngleMargin)){
      armMotor.set(0);
    }
    else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
      armMotor.set(driveTrainConstants.armMotorSpeed);
    }
    else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
      armMotor.set(-driveTrainConstants.armMotorSpeed);
    }
    else {
      armMotor.set(0);
    }
    // armMotor.set(armController.calculate(armMotorEncoder.getPosition(), neededEncoderCounts));
    if (Math.abs(neededEncoderCounts -armMotorEncoder.getPosition()) < driveTrainConstants.armAngleMargin){
      autonCompletedRotating = true;
    }
  }

   public void autonToggleArmExtension(){
    toggleArm();
   } 
   
   public void toggleClaw(boolean auto){
    if (m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON) || auto){
        clawSolenoid.toggle();
    }
  } 
  public void testExtension(){
    if(m_stick.getRawButtonPressed(robotConstants.EXTEND_TEST_BUTTON)){
      toggleArm();
    }
  }
  public void testArm(){
    if (getEncoderPosition()>driveTrainConstants.armLowerLimit+driveTrainConstants.armAngleMargin && getEncoderPosition()<driveTrainConstants.armUpperLimit-driveTrainConstants.armAngleMargin){
      if (m_stick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN)){
          armMotor.set(-0.9);
      }
      else if (m_stick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP)){
          armMotor.set(0.90);
      }
      else{
        armMotor.set(0);
      }
  } 
  }
}
