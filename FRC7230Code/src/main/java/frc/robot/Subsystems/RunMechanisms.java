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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
public class RunMechanisms {
    public boolean rotateComplete;
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawSolenoid = Mechanisms.clawSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    private final DutyCycleEncoder armMotorEncoder = Mechanisms.armMotorEncoder;
    private final PIDController armController = Mechanisms.armController;
    
  private double neededEncoderCounts;
  public boolean buttonPressed = false;
  private boolean prevButton = false;
  private boolean needExtend;
  public boolean autonCompletedRotating = false;
  public String armAngle = "Ground Pickup";
  public boolean completedRotating = false;

  public double getEncoderPosition(){
    double pos = armMotorEncoder.getAbsolutePosition()*360+100;
    if (pos > 200){
      return pos-360;
    }
    else {
      return pos;
    }
  }

  public void toggleArm(){
    if (Mechanisms.lowerLimitSwitch.get() && (getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition() < driveTrainConstants.armLowerExtension) || (getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
      armSolenoid.toggle();
      System.out.println("setting toggle");
    }
    else{
      armSolenoid.set(false);
      System.out.println("setting off2");
    }
  }
  // overloaded
  public void toggleArm(boolean value){
    if (Mechanisms.lowerLimitSwitch.get() && (getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition() < driveTrainConstants.armLowerExtension) || (getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
      armSolenoid.set(value);
      System.out.println("setting");
    }
    else{
      armSolenoid.set(false);
      System.out.println("setting off");
    }
  }
  public void checkArm(){
    if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition() < driveTrainConstants.armLowerExtension) && !(getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
      toggleArm(false);
    }
    System.out.println(getEncoderPosition());
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
      // System.out.println("button pressed");
    if ((!Mechanisms.upperLimitSwitch.get() && getEncoderPosition() < neededEncoderCounts) || (!Mechanisms.lowerLimitSwitch.get() && getEncoderPosition() > neededEncoderCounts)){
      armMotor.set(0);
      System.out.println("limit kill");
    }
    else if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition()<driveTrainConstants.armUpperLimit)){
      armMotor.set(0);
      System.out.println("oob kill");
    }
    else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
      armMotor.set(-driveTrainConstants.armMotorSpeed);
      System.out.println("up");
    }
    else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
      armMotor.set(driveTrainConstants.armMotorSpeed);
      System.out.println("down");
    }
    else {
      armMotor.set(0);
      System.out.println("other kill");
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
      if (Math.abs(neededEncoderCounts - getEncoderPosition()) < driveTrainConstants.armAngleMargin){
        if (needExtend && !completedRotating){
          toggleArm(true);
          System.out.println("sending");
          System.out.println(getEncoderPosition());
        }
        else if (needExtend && !prevButton){
          toggleArm(false);
        }
        completedRotating = true;
        
      }
      // else if (completedRotating && Math.abs(neededEncoderCounts - getEncoderPosition()) > driveTrainConstants.armAngleMargin){
      //   if ((!Mechanisms.upperLimitSwitch.get() && getEncoderPosition() < neededEncoderCounts) || (!Mechanisms.lowerLimitSwitch.get() && getEncoderPosition() > neededEncoderCounts)){
      //     armMotor.set(0);
      //     System.out.println("limit kill");
      //   }
      //   else if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition()<driveTrainConstants.armUpperLimit)){
      //     armMotor.set(0);
      //     System.out.println("oob kill");
      //   }
      //   else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
      //     armMotor.set(-driveTrainConstants.armMotorSpeed);
      //     System.out.println("up");
      //   }
      //   else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
      //     armMotor.set(driveTrainConstants.armMotorSpeed);
      //     System.out.println("down");
      //   }
      //   else {
      //     armMotor.set(0);
      //     System.out.println("other kill");
      //   }
      // }
      else if (needExtend && neededEncoderCounts == driveTrainConstants.lowPickupAngleEncoderCounts && Math.abs(getEncoderPosition() - driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.groundPickupAngleMargin)<driveTrainConstants.armAngleMargin){
        toggleArm(true);
        System.out.println("firing");
      }
      else{
        completedRotating = false;
        if (getEncoderPosition() > driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.groundPickupAngleMargin){
          toggleArm(false);
          // if (getEncoderPosition() < driveTrainConstants.lowPickupAngleEncoderCounts -)
          // armMotor.set(-driveTrainConstants.armMotorSpeed);
          System.out.println("out of position");
        }
      }
    }   
    else {
      armMotor.set(0);
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
    if ((!Mechanisms.upperLimitSwitch.get() && getEncoderPosition() < neededEncoderCounts) || (!Mechanisms.lowerLimitSwitch.get() && getEncoderPosition() > neededEncoderCounts)){
      armMotor.set(0);
      System.out.println("limit kill");
    }
    else if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition()<driveTrainConstants.armUpperLimit)){
      armMotor.set(0);
      System.out.println("oob kill");
    }
    else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
      armMotor.set(-driveTrainConstants.armMotorSpeed);
      System.out.println("up");
    }
    else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
      armMotor.set(driveTrainConstants.armMotorSpeed);
      System.out.println("down");
    }
    else {
      armMotor.set(0);
      System.out.println("other kill");
    }
    // armMotor.set(armController.calculate(armMotorEncoder.getPosition(), neededEncoderCounts));
    if (Math.abs(neededEncoderCounts - getEncoderPosition()) < driveTrainConstants.armAngleMargin){
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
    
    if ( (!Mechanisms.upperLimitSwitch.get() && m_stick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP)) || (!Mechanisms.lowerLimitSwitch.get()&& m_stick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN))){
      armMotor.set(0);
      System.out.println("limit kill");
      System.out.println("upper: "+Mechanisms.upperLimitSwitch.get());
      System.out.println("lower: "+Mechanisms.lowerLimitSwitch.get());
    }
    else if (getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition()<driveTrainConstants.armUpperLimit){
      if (m_stick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN)){
          armMotor.set(0.9);
          System.out.println("down");
      }
      else if (m_stick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP)){
          armMotor.set(-0.90);
          System.out.println("up");
      }
      else{
        armMotor.set(0);
        // System.out.println("no button kill");
      }
    } 
    else {
      armMotor.set(0);
      System.out.println("oob kill");
    }
  }
}
