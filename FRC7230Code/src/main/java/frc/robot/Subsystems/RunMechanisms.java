package frc.robot.Subsystems;

import frc.robot.Limelight;
import frc.robot.Mechanisms;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
public class RunMechanisms {
    public boolean rotateComplete;
    private CANSparkMax armMotor = Mechanisms.armMotor; 
    private Solenoid armSolenoid = Mechanisms.armSolenoid, clawSolenoid = Mechanisms.clawSolenoid;
    private Joystick m_stick = Mechanisms.mechanismsJoystick;
    private final DutyCycleEncoder armMotorEncoder = Mechanisms.armMotorEncoder;
    // private final PIDController armController = Mechanisms.armController;
  private boolean inTimer = false;
    private Timer upTimer = new Timer();
  private double neededEncoderCounts;
  public boolean buttonPressed = false;
  private boolean prevButton = false;
  private boolean needExtend;
  public boolean autonCompletedRotating = false;
  public String armAngle = "Ground Pickup";
  public boolean completedRotating = false;

  public double getEncoderPosition(){
    double pos = armMotorEncoder.getAbsolutePosition()*360-50;
    if (pos > 300){
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
    if (!(getEncoderPosition() < driveTrainConstants.armLowerExtension) && !(getEncoderPosition()<driveTrainConstants.armUpperLimit && getEncoderPosition()>driveTrainConstants.armUpperRetraction)){
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
      completedRotating = false;
    }
    else if ((getEncoderPosition()<driveTrainConstants.armLowerLimit && neededEncoderCounts<getEncoderPosition()) || (getEncoderPosition()>driveTrainConstants.armUpperLimit && neededEncoderCounts>getEncoderPosition())){
      armMotor.set(0);
      System.out.println("oob kill");
      completedRotating = false;
    }
    else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
      armMotor.set(-driveTrainConstants.armMotorSpeed);
      System.out.println("up");
      completedRotating = false;
    }
    else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
      armMotor.set(driveTrainConstants.armMotorSpeed);
      System.out.println("down");
      completedRotating = false;
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
        // System.out.println(completedRotating);
        // System.out.println(needExtend);
        if (neededEncoderCounts < 100){
          armMotor.set(-0.0125);
        }
        else {
          armMotor.set(0.0125);
        }
        if (needExtend && !completedRotating){
          toggleArm(true);
          System.out.println("sending");
          System.out.println(getEncoderPosition());
        }
        else if (needExtend && !prevButton){
          if (neededEncoderCounts != driveTrainConstants.lowPickupAngleEncoderCounts){
            toggleArm(false);
            inTimer=false;
          }
          else {
            upTimer.reset();
            upTimer.start();
            // if (getEncoderPosition() > driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.armAngleMargin){
            inTimer=true;
            // }
          }
        }
        else {
          // inTimer=false;
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
      else if (needExtend && neededEncoderCounts == driveTrainConstants.lowPickupAngleEncoderCounts && Math.abs(getEncoderPosition() - driveTrainConstants.lowPickupAngleEncoderCounts - driveTrainConstants.groundPickupAngleMargin)<driveTrainConstants.armAngleMargin){
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
      
      if (upTimer.get()<0.75 && inTimer){
        armMotor.set(-driveTrainConstants.armMotorSpeed);
        }
        else if (inTimer && upTimer.get()<1.0){
        armMotor.set(-driveTrainConstants.armMotorSpeed/2);
        }
        else if (inTimer && upTimer.get()<1.25){
        armMotor.set(0);
        }
        else if (inTimer && upTimer.get()<1.75){
          toggleArm(false);
        }
        else if (inTimer){
          
          inTimer = false;
        }

    }   
    else if (!Mechanisms.driveJoystick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN) && !Mechanisms.driveJoystick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP)){
      armMotor.set(0);
    }
    // else if (inTimer){
      
    //   inTimer = false;
    // }
// passive rotation option and active extension
    // if (buttonPressed && completedRotating && !prevButton){
    //   toggleArm(true);
    // }
    // if (!completedRotating){
    //   toggleArm(false);
    // }
    
    // if ((!Mechanisms.upperLimitSwitch.get() && getEncoderPosition() < neededEncoderCounts) || (!Mechanisms.lowerLimitSwitch.get() && getEncoderPosition() > neededEncoderCounts)){
    //   armMotor.set(0);
    //   System.out.println("limit kill");
    // }
    // else if (!(getEncoderPosition()>driveTrainConstants.armLowerLimit && getEncoderPosition()<driveTrainConstants.armUpperLimit)){
    //   armMotor.set(0);
    //   System.out.println("oob kill");
    // }
    // else if (getEncoderPosition() < neededEncoderCounts - driveTrainConstants.armAngleMargin) {
    //   armMotor.set(-driveTrainConstants.armMotorSpeed);
    //   System.out.println("up");
    // }
    // else if (getEncoderPosition() > neededEncoderCounts + driveTrainConstants.armAngleMargin){
    //   armMotor.set(driveTrainConstants.armMotorSpeed);
    //   System.out.println("down");
    // }
    // else {
    //   armMotor.set(0);
    //   System.out.println("other kill");
    // }
      
    //   if (Math.abs(neededEncoderCounts - getEncoderPosition()) < driveTrainConstants.armAngleMargin){
    //     if (buttonPressed && needExtend && !prevButton){
    //       toggleArm();
    //     }
    //     completedRotating = true;
        
    //   }
    //   // else if (needExtend && neededEncoderCounts == driveTrainConstants.lowPickupAngleEncoderCounts && Math.abs(getEncoderPosition() - driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.groundPickupAngleMargin)<driveTrainConstants.armAngleMargin){
    //   //   toggleArm(true);
    //   //   System.out.println("firing");
    //   // }
    //   else{
    //     completedRotating = false;
    //     if (buttonPressed){
    //     toggleArm(false);
    //     }
    //     if (getEncoderPosition() > driveTrainConstants.lowPickupAngleEncoderCounts + driveTrainConstants.groundPickupAngleMargin){
    //       toggleArm(false);
    //       // if (getEncoderPosition() < driveTrainConstants.lowPickupAngleEncoderCounts -)
    //       // armMotor.set(-driveTrainConstants.armMotorSpeed);
    //       System.out.println("out of position");
    //     }
    //   }
    
    prevButton = buttonPressed;
  }

  public boolean autonRotateArmToAngle(double neededEncoderCounts){
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
    else if ((getEncoderPosition()<driveTrainConstants.armLowerLimit && neededEncoderCounts<getEncoderPosition()) || (getEncoderPosition()>driveTrainConstants.armUpperLimit && neededEncoderCounts>getEncoderPosition())){
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
    System.out.println(getEncoderPosition());
    // armMotor.set(armController.calculate(armMotorEncoder.getPosition(), neededEncoderCounts));
    if (Math.abs(neededEncoderCounts - getEncoderPosition()) < driveTrainConstants.armAngleMargin){
      autonCompletedRotating = true;
      return true;
    }
    else{
      return false;
    }
  }

  public void autonToggleArmExtension(){
    toggleArm();
   } 
   
   public void autonToggleArmExtension(boolean val){
    toggleArm(val);
   } 
   
   public void toggleClaw(boolean auto){
    if (auto || m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON)){
        clawSolenoid.toggle();
    }
  } 
  public void toggleClaw(boolean auto, boolean state){
   if (auto || m_stick.getRawButtonPressed(robotConstants.CLAW_TOGGLE_BUTTON)){
       clawSolenoid.set(state);
   }
 } 
  public void testExtension(Joystick stick){
    if(stick.getRawButtonPressed(robotConstants.EXTEND_TEST_BUTTON)){
      toggleArm();
    }
  }
  public void testArm(Joystick stick){
    
    if ( (!Mechanisms.upperLimitSwitch.get() && stick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP)) || (!Mechanisms.lowerLimitSwitch.get()&& stick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN))){
      armMotor.set(0);
      System.out.println("limit kill");
      System.out.println("upper: "+Mechanisms.upperLimitSwitch.get());
      System.out.println("lower: "+Mechanisms.lowerLimitSwitch.get());
    }
    else if ( !(getEncoderPosition()<driveTrainConstants.armLowerLimit && stick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN)) && !(getEncoderPosition()>driveTrainConstants.armUpperLimit && stick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP))){
      if (stick.getRawButton(robotConstants.ARM_TEST_BUTTON_DOWN)){
          armMotor.set(driveTrainConstants.armMotorSpeed);
          System.out.println("down");
      }
      else if (stick.getRawButton(robotConstants.ARM_TEST_BUTTON_UP)){
          armMotor.set(-driveTrainConstants.armMotorSpeed);
          System.out.println("up");
      }
      else if (!m_stick.getRawButton(robotConstants.SHELF_PICKUP_BUTTON) && !m_stick.getRawButton(robotConstants.GROUND_PICKUP_BUTTON) && !m_stick.getRawButton(robotConstants.HIGH_SCORE_BUTTON) && !m_stick.getRawButton(robotConstants.MID_SCORE_BUTTON) && !m_stick.getRawButton(robotConstants.LOW_SCORE_BUTTON)){
        armMotor.set(0);
        System.out.println("no button kill");
      }
    } 
    else {
      armMotor.set(0);
      System.out.println("oob kill");
    }
  }
}
