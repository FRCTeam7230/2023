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
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class RunMechanisms {
    public boolean rotateComplete;
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawSolenoid = Mechanisms.clawSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    public final SparkMaxAbsoluteEncoder armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // public final SparkMaxPIDController armController = armMotor.getPIDController();armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    public final PIDController armController = new PIDController(driveTrainConstants.kP, driveTrainConstants.kI, driveTrainConstants.kD);
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
  public void rotateArmToAngle(){
    armMotorEncoder.setPositionConversionFactor(driveTrainConstants.rotationsToDegrees);  
    // armController.setP(driveTrainConstants.kP);
    // armController.setI(driveTrainConstants.kI);
    // armController.setD(driveTrainConstants.kD);
    // armController.setFF(driveTrainConstants.kFF);
    if (m_stick.getRawButton(robotConstants.SHELF_PICKUP_BUTTON)){
      buttonPressed = true;
      needExtend = true;
      armAngle = "Shelf";
      neededEncoderCounts = driveTrainConstants.shelfAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.HIGH_SCORE_BUTTON)) {
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
    if (m_stick.getRawButton(robotConstants.MID_SCORE_BUTTON)) {
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
    if (m_stick.getRawButton(robotConstants.GROUND_PICKUP_BUTTON)) {
      buttonPressed = true;
      needExtend = true;
      armAngle = "Ground Pickup";
      neededEncoderCounts = driveTrainConstants.lowPickupAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.LOW_SCORE_BUTTON)){
      buttonPressed = true;
      needExtend = false;
      armAngle = "Low Scoring";
      neededEncoderCounts = driveTrainConstants.lowScoreAngleEncoderCounts;
    }

    if (buttonPressed){
      
      if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
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
      if (neededEncoderCounts == armMotorEncoder.getPosition()){
        completedRotating = true;
        if (!prevButton && needExtend){
          armSolenoid.toggle();
        }
      }
      else{
        completedRotating = false;
        armSolenoid.set(false);
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
    
    Mechanisms.armPID.setReference(autonEncoderCounts, ControlType.kPosition);
    if (neededEncoderCounts == armMotorEncoder.getPosition()){
      autonCompletedRotating = true;
    }
  }

   public void autonToggleArmExtension(){
        armSolenoid.toggle();
   } 
   public void toggleClaw(boolean auto){
    if (m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON) || auto){
        clawSolenoid.toggle();
    }
  } 
  public void testExtension(){
    if(m_stick.getRawButtonPressed(robotConstants.EXTEND_TEST_BUTTON)){
      armSolenoid.toggle();
    }
  }
  public void testArm(boolean auto){
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
