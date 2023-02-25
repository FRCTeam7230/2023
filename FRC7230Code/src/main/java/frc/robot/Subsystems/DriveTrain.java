package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
import frc.robot.Limelight;
import frc.robot.Mechanisms;




public class DriveTrain {
    
    private DriveSubsystem m_robotDrive;
    private RunMechanisms m_runMechanisms;
    private Joystick m_stick;
    private boolean surpassedMargin;
    private boolean surpassedMargin2;
    private boolean swapState = false, prevState = false;
    private int invertAxis = 1;
    private double speedY = 0.0;
    private double speedX = 0.0;
    private double speedLimitChangeX = 0.0;
    private double speedLimitChangeY = 0.0;
    private double rateOfSpeedYChange = 0.0;
    private double rateOfSpeedXChange = 0.0;
    private boolean prevDrive = false, nowDrive = false;
    private double gyroAngle;
    private double error;
    private double errorToTarget;
    private boolean button1State;
    private boolean button2State;
    private boolean button3State;
    private boolean button4State;
    private boolean button5State;
    private boolean button6State;
    private boolean button7State;
    private boolean button8State;
    private boolean button9State;
    private boolean button10State;
    private boolean button11State;
    private boolean button12State;
    private boolean prevButton3 = false;
    private boolean prevButton5 = false;
    private boolean prevButton6 = false;
    private boolean prevButton8 = false;
    private boolean prevButton11 = false;
    private boolean prevButton12 = false;

    //Vision:
    private double[] gamePieceAreas;
    private double angleToTarget;
    private boolean manualLayout = true; // if true - 1st layout (manual), false - 2nd (smart)
    private boolean continueMoving;

    private final Encoder armMotorEncoder = new Encoder(7, 8, 9);
    private boolean driveModified;
    public DriveTrain(DriveSubsystem subsystem, RunMechanisms runMechanisms, Joystick stick){
        m_robotDrive = subsystem;
        m_runMechanisms = runMechanisms;
        m_stick = stick;
       
        surpassedMargin = false;
    }

    public double getArmMotorAngle(){
        return (armMotorEncoder.get()*driveTrainConstants.degreesToArmEncoderCounts);
    }

    public void drive(boolean tank){
        speedLimitChangeX = 0.0;
        speedLimitChangeY = 0.0;
        double y = Math.pow(m_stick.getRawAxis(0),1);
        double x = Math.pow(m_stick.getRawAxis(1),1);
        y *= Math.abs(y);
        x *= Math.abs(x);
        int invertChangeY = 1;
        int invertChangeX = 1;
        // if x and y are negative
        if (y < -1 * driveTrainConstants.deadZone || (speedY<0 && Math.abs(y)<driveTrainConstants.deadZone)){
            invertChangeY = -1;
        }
        if (x < -1 * driveTrainConstants.deadZone || (speedX<0 && Math.abs(x)<driveTrainConstants.deadZone)){
            invertChangeX = -1;
        }
        // if y is being driven, and has not yet reached speed target
        if(driveTrainConstants.deadZone < Math.abs(y) && Math.abs(y) > Math.abs(speedY)) {
            speedY += invertChangeY * rateOfSpeedYChange;
            rateOfSpeedYChange += driveTrainConstants.accelY;
            nowDrive = true;
        }
        // if y is not being driven and is slow
        else if (driveTrainConstants.deadZone>=Math.abs(y) && Math.abs(speedY)<driveTrainConstants.dropOff){
            speedY=0;
        }
        // if y is not being driven and is fast
        else if (driveTrainConstants.deadZone>=Math.abs(y) && Math.abs(speedY)>=driveTrainConstants.dropOff){
            speedY-=driveTrainConstants.decelY*invertChangeY;
        }
        // if x being driven
        if(driveTrainConstants.deadZone < Math.abs(x) && Math.abs(x) > Math.abs(speedX)) {
            speedX += invertChangeX * rateOfSpeedXChange;
            rateOfSpeedXChange += driveTrainConstants.accelX;
            nowDrive = true;
            
        }
        // if x not being driven but is slow
        else if (driveTrainConstants.deadZone>=Math.abs(x) && Math.abs(speedX)<driveTrainConstants.dropOff){
            speedX=0;
        }
        // if x not being driven but is fast
        else if (driveTrainConstants.deadZone>=Math.abs(x) && Math.abs(speedX)>=driveTrainConstants.dropOff){
            speedX-=driveTrainConstants.decelY*invertChangeX;
        }
        if (Math.abs(x)<driveTrainConstants.deadZone && Math.abs(y)<driveTrainConstants.deadZone)
        {
            nowDrive = false;
        }
        // when started driving, reset speed changes and add initial speed
        if (nowDrive && !prevDrive){
            rateOfSpeedXChange = 0;
            rateOfSpeedYChange = 0;
            if (Math.abs(x)>driveTrainConstants.deadZone){
                speedX += driveTrainConstants.initSpeed * invertChangeX;
            }
            if (Math.abs(y)>driveTrainConstants.deadZone){
                speedY += driveTrainConstants.initSpeed * invertChangeY;
            }
        }
        speedX=x;
        speedY=y;

        if(m_stick.getRawButton(robotConstants.SPEED_BUTTON)){
            speedY *= driveTrainConstants.zoomFactor;
            speedX *= driveTrainConstants.zoomFactor;
            speedLimitChangeX = 0.1;
            speedLimitChangeY = 0.1;
        }
        
        if(m_stick.getRawButton(robotConstants.SLOW_BUTTON)){
            speedX=x;
            speedY=y;
            speedLimitChangeX = -0.2;
            speedLimitChangeY = -0.2;
            speedY*=driveTrainConstants.slowFactor;
            speedX*=driveTrainConstants.slowFactor; 
        }
        // IMPORTANT
        // I DONT KNOW WHY BUT X AND Y LIMITS HERE ARE SWITCHED
        if (speedX > 0){
            speedX = Math.min(speedX, driveTrainConstants.limitX + speedLimitChangeX);
        }
        else {
            speedX = Math.max(speedX, -driveTrainConstants.limitX - speedLimitChangeX);
        }
        if (speedY > 0){
            speedY = Math.min(speedY, driveTrainConstants.limitY + speedLimitChangeY);
        }
        else {
            speedY = Math.max(speedY, -driveTrainConstants.limitY - speedLimitChangeY);
        }
        
        // inverting
        swapState = m_stick.getRawButton(robotConstants.REVERSE_BUTTON);
        if (swapState == true && prevState == false){
            invertAxis *= -1;
            prevState = true;
        }
        else if (!swapState){
            prevState = false;
        }
        prevState = swapState;
        prevDrive = nowDrive;

        if (!tank && !driveModified){
            DriverStation.reportWarning(Double.toString(invertAxis * speedX), false);
            DriverStation.reportWarning(Double.toString(invertAxis * speedY), false);
            DriverStation.reportWarning("\n", false);
            m_robotDrive.arcadeDrive(1 * invertAxis * (speedY), invertAxis *(speedX));
        }


        // else if (error<-0.3 && error <- driveTrainConstants.smartAngleMargin && surpassedMargin){
        //     m_robotDrive.drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
        //     System.out.println("forward");
        // }
        // else{
        //     m_robotDrive.drive(0, 0);

        //Switching target - cube, or cone 
        button5State = m_stick.getRawButtonPressed(robotConstants.GAME_PIECE_TOGGLE_BUTTON);
        if (button5State){ 
            Limelight.coneTarget = !Limelight.coneTarget;
            gamePieceAreas = Limelight.updateTarget();
        }
        
        
        // Switching joystick layout from manual to smart
        button6State = m_stick.getRawButtonPressed(robotConstants.MANUAL_SMART_TOGGLE_BUTTON);
        if (button6State) {
            manualLayout = !manualLayout;
            
        }

        
        // Smart control
        if (!manualLayout && m_runMechanisms.completedRotating) {
            
        
            if (m_stick.getRawButton(robotConstants.ORIENT_SHELF_PICKUP_BUTTON)) {
                continueMoving = Limelight.procedMoving(gamePieceAreas[1]);
            }
            if (m_stick.getRawButton(robotConstants.ORIENT_GROUND_PICKUP_BUTTON)) {
                continueMoving = Limelight.procedMoving(gamePieceAreas[0]);
            }
            if (m_stick.getRawButton(robotConstants.ORIENT_HIGH_TARGET_BUTTON)) {
                continueMoving = Limelight.updateTape(true, driveTrainConstants.tapeScreenAreas);
            }
            if (m_stick.getRawButton(robotConstants.ORIENT_MID_TARGET_BUTTON)) {
                continueMoving = Limelight.updateTape(false, driveTrainConstants.tapeScreenAreas);
            }
            if (m_runMechanisms.buttonPressed) {
                angleToTarget = Limelight.getTargetAngleX(); // getting angle to drive to the robot

                if (continueMoving) { // checking if target is enouph close to the robot. If not, continue moving
                    if (angleToTarget>0 && angleToTarget>driveTrainConstants.smartAngleMarginVision){
                        driveModified = true;
                        m_robotDrive.drive(driveTrainConstants.smartSpeedVision/2, -driveTrainConstants.smartSpeedVision);
                    }
                    else if (angleToTarget<0 && angleToTarget<-driveTrainConstants.smartAngleMarginVision){
                        driveModified = true;
                        m_robotDrive.drive(-driveTrainConstants.smartSpeedVision, driveTrainConstants.smartSpeedVision/2);
                    }
                    else if (Math.abs(angleToTarget) < driveTrainConstants.smartAngleMarginVision){
                        driveModified = true;
                        m_robotDrive.drive(-driveTrainConstants.smartSpeedVision, -driveTrainConstants.smartSpeedVision);
                        //Mechanisms.armSolenoid.set(true);
                    }
                }
                else {
                    
                    // after robot arrive, it should rotate, extend arm and exterd+retrackt claw
                    driveModified = false;
                    
                    // if (armMotorEncoder.get() < driveTrainConstants.midAngleEncoderCounts) {
                    // Mechanisms.armMotor.set(0.5);
                    // }
                }
            }



    
            
            else if (m_stick.getRawButton(robotConstants.ORIENT_GROUND_PICKUP_BUTTON)) {
                
            }
            else if (m_stick.getRawButton(robotConstants.ORIENT_HIGH_TARGET_BUTTON)) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(7); 
                
            }
            else if (m_stick.getRawButton(robotConstants.ORIENT_MID_TARGET_BUTTON)) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(8); // change pipeline

            }
            else{
                driveModified = false;
                Limelight.continueMoving = true;
            }
        }


        button3State = m_stick.getRawButton(robotConstants.BALANCING_BUTTON);
        if (button3State){
            if (!prevButton3){
                surpassedMargin = false;
                surpassedMargin2 = false;
            }
            if (Math.abs(error)>driveTrainConstants.smartAngleMargin && !surpassedMargin){
                surpassedMargin = true;
                DriverStation.reportWarning("MARGIN PASSED", false);
            }
            if (Math.abs(error)>driveTrainConstants.smartAngleMargin2 && surpassedMargin && !surpassedMargin2){
                surpassedMargin2 = true;
                DriverStation.reportWarning("MARGIN 2 PASSED", false);
            }
            gyroAngle = Mechanisms.gyro.getPitch();
            error = driveTrainConstants.targetAngle - gyroAngle;
            // System.out.println(gyroAngle);
            System.out.println(error);
            
            if (error > driveTrainConstants.smartAngleMargin || (!surpassedMargin && !surpassedMargin2 && error>0.3)){
                m_robotDrive.drive(driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed);
                System.out.println("forward");
            }
            else if (error>0.3 && error > driveTrainConstants.smartAngleMargin && surpassedMargin){
                m_robotDrive.drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
                System.out.println("forward");
            }
            else if (error < - driveTrainConstants.smartAngleMargin || (!surpassedMargin&& !surpassedMargin2  && error<-0.3)){
                m_robotDrive.drive(-driveTrainConstants.smartSpeed, -driveTrainConstants.smartSpeed);
                System.out.println("backward");
            }
            else if (error<-0.3 && error <- driveTrainConstants.smartAngleMargin && surpassedMargin){
                m_robotDrive.drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
                System.out.println("forward");
            }
            else{
                m_robotDrive.drive(0, 0);
            }

        // prevButton3 = button3State;
        // button3State = m_stick.getRawButton(robotConstants.SMART_ORIENT_CONE_MID_ANGLE);
        // if (button3State){
        //     NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(7);
        //     double angle = Limelight.getTargetAngleX();
        //     if (armMotorEncoder.get() < driveTrainConstants.midAngleEncoderCounts) {
        //         Mechanisms.armMotor.set(0.5);
        //     }
        //     if (angle>0 && angle>driveTrainConstants.smartAngleMarginVision){
        //       driveModified = true;
        //       m_robotDrive.drive(driveTrainConstants.smartSpeedVision/2, -driveTrainConstants.smartSpeedVision);
        //     }
        //     else if (angle<0 && angle<-driveTrainConstants.smartAngleMarginVision){
        //       driveModified = true;
        //       m_robotDrive.drive(-driveTrainConstants.smartSpeedVision, driveTrainConstants.smartSpeedVision/2);
        //     }
        //     else if (Math.abs(angle) < driveTrainConstants.smartAngleMarginVision){
        //       Mechanisms.armSolenoid.set(true);
        //     }
        // }
        // else{
        //   driveModified = false;
        // }

        // button4State = m_stick.getRawButton(robotConstants.SMART_ORIENT_CONE_HIGH_ANGLE);
        // if (button4State){
            
        //     NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(7);
        //     double angle = Limelight.getTargetAngleX();
        // if (armMotorEncoder.get() < driveTrainConstants.highAngleEncoderCounts) {
        //         Mechanisms.armMotor.set(0.5);
        //     }
        //     else if (angle<0 && angle<-driveTrainConstants.smartAngleMarginVision){
        //       driveModified = true;
        //       m_robotDrive.drive(-driveTrainConstants.smartSpeedVision, driveTrainConstants.smartSpeedVision/2);
        //     }
        //     else if (Math.abs(angle) < driveTrainConstants.smartAngleMarginVision){
        //       Mechanisms.armSolenoid.set(true);
        //     }
        // }
        button11State = m_stick.getRawButton(robotConstants.ENCODER_TEST_BUTTON);
        if (button11State){
            if (!prevButton11){
                m_robotDrive.resetEncoders();
                System.out.println("kiwi");
            }
            // if (m_robotDrive.getAverageEncoderDistance() < driveTrainConstants.midScoreMeters) {
            //     m_robotDrive.drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
            //     //while holding the button, drive back 1 meter
            // } 
            System.out.println("LEFT: " + m_robotDrive.getLeftDistance());
            System.out.println("RIGHT: " + m_robotDrive.getRightDistance());
            m_robotDrive.lController.setReference(m_robotDrive.m_leftEncoder.getPosition()+ driveTrainConstants.testDistance * driveTrainConstants.kEncoderMetersPerMotorRev, CANSparkMax.ControlType.kPosition);
            m_robotDrive.rController.setReference(m_robotDrive.m_rightEncoder.getPosition()+driveTrainConstants.testDistance * driveTrainConstants.kEncoderMetersPerMotorRev, CANSparkMax.ControlType.kPosition);
        }   
        else{
          driveModified = false;
        } 
        prevButton11 = button11State;
        
    }
}
}




