package frc.robot.Subsystems;

import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
import frc.robot.Limelight;
import frc.robot.Mechanisms;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class DriveTrain {
    
    private DriveSubsystem m_robotDrive;
    private RunMechanisms m_runMechanisms;
    private Joystick m_stick;
    private Joystick d_stick;
    private boolean surpassedMargin;
    private boolean surpassedMargin2;
    private boolean swapState = false, prevState = false;
    public static int invertAxis = -1;
    public double speedY = 0.0;
    public double speedX = 0.0;
    public double speedLimitChangeX = 0.0;
    public double speedLimitChangeY = 0.0;
    private double rateOfSpeedYChange = 0.0;
    private double rateOfSpeedXChange = 0.0;
    private boolean prevDrive = false, nowDrive = false;
    private double gyroAngle;
    private double gyroError;
    private boolean button3State;
    
    private boolean button5State;
    private boolean button6State;
    
    
    private boolean prevButton3 = false;
    public boolean manualLayout = true; // layout of the joystick - manual or smart
    private boolean driveModified;
    private boolean pickup;

    //Vision:
    private double[] gamePieceAreas;
    private double angleToTarget;
    private boolean continueMoving;

   
    public DriveTrain(DriveSubsystem subsystem, RunMechanisms runMechanisms, Joystick stick1, Joystick stick2){
        m_robotDrive = subsystem;
        m_runMechanisms = runMechanisms;
        m_stick = stick1;
        d_stick = stick2;
        surpassedMargin = false;
    }
    

    public void drive(boolean tank){
        // speedLimitChangeX = Robot.speedLimitChangeX;
        // speedLimitChangeY = Robot.speedLimitChangeY;
        double y = Math.pow(d_stick.getRawAxis(0),1);
        double x = Math.pow(d_stick.getRawAxis(1),1);
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

        if(d_stick.getRawButton(robotConstants.SPEED_BUTTON)){
            speedY *= driveTrainConstants.zoomFactor;
            speedX *= driveTrainConstants.zoomFactor;
            speedLimitChangeX = 0.1;
            speedLimitChangeY = 0.1;
        }
        
        if(d_stick.getRawButton(robotConstants.SLOW_BUTTON)){
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
        swapState = d_stick.getRawButton(robotConstants.REVERSE_BUTTON);
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
            m_robotDrive.arcadeDrive(invertAxis * (speedY), invertAxis *(speedX));
        }

        //Switching target - cube, or cone 
        button5State = d_stick.getRawButtonPressed(robotConstants.GAME_PIECE_TOGGLE_BUTTON);
        if (button5State){ 
            Limelight.coneTarget = !Limelight.coneTarget;
            gamePieceAreas = Limelight.updateTarget();
        }        
        
        // Switching joystick layout from manual to smart
        button6State = d_stick.getRawButtonPressed(robotConstants.MANUAL_SMART_TOGGLE_BUTTON);
        if (button6State) {
            manualLayout = !manualLayout;
        }

        // Smart control
        if (!manualLayout && m_runMechanisms.completedRotating) {
            if (m_stick.getRawButton(robotConstants.ORIENT_SHELF_PICKUP_BUTTON)) {
                continueMoving = Limelight.proceedMoving(gamePieceAreas[1]);
            }
            if (m_stick.getRawButton(robotConstants.ORIENT_GROUND_PICKUP_BUTTON)) {
                continueMoving = Limelight.proceedMoving(gamePieceAreas[0]);
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
                        if (pickup){
                            m_robotDrive.drive(-driveTrainConstants.smartSpeedVision, -driveTrainConstants.smartSpeedVision);
                        }    
                        else{
                            m_robotDrive.drive(driveTrainConstants.smartSpeedVision, driveTrainConstants.smartSpeedVision);
                        }   
                    }
                }
                else {
                    driveModified = false;       
                }
            }
        }


        button3State = d_stick.getRawButton(robotConstants.BALANCING_BUTTON);
        if (button3State){
            if (!prevButton3){
                surpassedMargin = false;
                surpassedMargin2 = false;
            }
            if (Math.abs(gyroError)>driveTrainConstants.smartAngleMargin && !surpassedMargin){
                surpassedMargin = true;
                DriverStation.reportWarning("MARGIN PASSED", false);
            }
            if (Math.abs(gyroError)>driveTrainConstants.smartAngleMargin2 && surpassedMargin && !surpassedMargin2){
                surpassedMargin2 = true;
                DriverStation.reportWarning("MARGIN 2 PASSED", false);
            }
            gyroAngle = Mechanisms.gyro.getPitch();
            gyroError = driveTrainConstants.targetAngle - gyroAngle;
            // System.out.println(gyroAngle);
            System.out.println(gyroError);
            
            if (gyroError > driveTrainConstants.smartAngleMargin || (!surpassedMargin && !surpassedMargin2 && gyroError>0.3)){
                m_robotDrive.drive(driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed);
                System.out.println("forward");
            }
            else if (gyroError>0.3 && gyroError > driveTrainConstants.smartAngleMargin && surpassedMargin){
                m_robotDrive.drive(-driveTrainConstants.slowSmartSpeed, -driveTrainConstants.slowSmartSpeed);
                System.out.println("forward");
            }
            else if (gyroError < - driveTrainConstants.smartAngleMargin || (!surpassedMargin&& !surpassedMargin2  && gyroError<-0.3)){
                m_robotDrive.drive(-driveTrainConstants.smartSpeed, -driveTrainConstants.smartSpeed);
                System.out.println("backward");
            }
            else if (gyroError<-0.3 && gyroError <- driveTrainConstants.smartAngleMargin && surpassedMargin){
                m_robotDrive.drive(driveTrainConstants.slowSmartSpeed, driveTrainConstants.slowSmartSpeed);
                System.out.println("forward");
            }
            else{
                m_robotDrive.drive(0, 0);
            }
        }
    }
}




