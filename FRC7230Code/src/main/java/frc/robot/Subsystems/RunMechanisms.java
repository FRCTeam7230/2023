package frc.robot.Subsystems;

import frc.robot.Mechanisms;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class RunMechanisms {
    public boolean rotateComplete;
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawRightSolenoid = Mechanisms.clawRightSolenoid, clawLeftSolenoid = Mechanisms.clawLeftSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    private final SparkMaxAbsoluteEncoder armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    public final SparkMaxPIDController armController = armMotor.getPIDController();
  private double encoderCounts;
  public boolean buttonPressed = false;
  private boolean prevButton = false;
  private boolean needExtend;
  public boolean autonCompletedRotating = false;
  
  public boolean completedRotating = false;


  public void rotateArmToAngle(){
    armMotorEncoder.setPositionConversionFactor(driveTrainConstants.rotationsToDegrees);
    armController.setP(driveTrainConstants.kP);
    armController.setI(driveTrainConstants.kI);
    armController.setD(driveTrainConstants.kD);
    armController.setFF(driveTrainConstants.kFF);
    if (m_stick.getRawButton(robotConstants.SHELF_PICKUP_BUTTON)){
      buttonPressed = true;
      needExtend = true;
      encoderCounts = driveTrainConstants.shelfAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.HIGH_SCORE_BUTTON)) {
      buttonPressed = true;
      needExtend = true;
      encoderCounts = driveTrainConstants.coneHighAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.MID_SCORE_BUTTON)) {
      buttonPressed = true;
      needExtend = false;
      encoderCounts = driveTrainConstants.coneMidAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.GROUND_PICKUP_BUTTON)) {
      buttonPressed = true;
      needExtend = true;
      encoderCounts = driveTrainConstants.lowPickupAngleEncoderCounts;
    }
    if (m_stick.getRawButton(robotConstants.LOW_SCORE_BUTTON)){
      buttonPressed = true;
      needExtend = false;
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
      // }
      Mechanisms.armPID.setReference(encoderCounts, ControlType.kPosition);
      if (encoderCounts == armMotorEncoder.getPosition()){
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
    if (encoderCounts == armMotorEncoder.getPosition()){
      autonCompletedRotating = true;
      
    }
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
