package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;
import frc.robot.Mechanisms;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class DriveTrain {

    private DriveSubsystem m_robotDrive;

    private Joystick m_stick;
    
    private boolean swapState = false, prevState = false;
    private int invertAxis = 1;
    private double speedY = 0.0;
    private double speedX = 0.0;
    private double speedLimitChangeX = 0.0;
    private double speedLimitChangeY = 0.0;
    private double rateOfSpeedYChange = 0.0;
    private double rateOfSpeedXChange = 0.0;
    private boolean prevDrive = false, nowDrive = false;
    private double targetAngle;
    private double gyroAngle;

    private double error;
    private boolean button3State;
    private boolean button9State;
    private boolean button11State;
    
    private boolean driveModified;
    public DriveTrain(DriveSubsystem subsystem, Joystick stick){
        m_robotDrive = subsystem;
        m_stick = stick;
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
        // button3State = m_stick.getRawButton(robotConstants.SMART_INTAKE_BUTTON);
        // if (button3State){
        //     Mechanisms.intakeSolenoid.set(button3State);
        //     Mechanisms.intakeMotor.set(ControlMode.PercentOutput, 0.65);
        //     Mechanisms.conveyorMotor.set(0.5);
        //     double angle = Mechanisms.vision.getAngleX();
        //     if (angle>0 && angle>driveTrainConstants.smartAngleMargin){
        //       driveModified = true;
        //       m_robotDrive.drive(driveTrainConstants.smartSpeed/2, -driveTrainConstants.smartSpeed);
        //     }
        //     else if (angle<0 && angle<-driveTrainConstants.smartAngleMargin){
        //       driveModified = true;
        //       m_robotDrive.drive(-driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed/2);
        //     }
        //     else if (Math.abs(angle) < driveTrainConstants.smartAngleMargin){
        //       driveModified = true;
        //       m_robotDrive.arcadeDrive(0, -driveTrainConstants.smartSpeed);
        //     }
        // }
        // else{
        //   driveModified = false;
        // }
        button9State = m_stick.getRawButton(robotConstants.BALANCING_BUTTON);
        if (button9State){
            gyroAngle = Mechanisms.gyro.getPitch();
            error = driveTrainConstants.targetAngle - gyroAngle;
            // System.out.println(gyroAngle);
            System.out.println(error);
            
            if (error > driveTrainConstants.smartAngleMargin){
                m_robotDrive.drive(driveTrainConstants.smartSpeed, driveTrainConstants.smartSpeed);
                SmartDashboard.putString("tilted forward", " driving back");
                System.out.println("forward");
            }
            else if (error < - driveTrainConstants.smartAngleMargin){
                m_robotDrive.drive(-driveTrainConstants.smartSpeed, -driveTrainConstants.smartSpeed);
                System.out.println("backward");
                SmartDashboard.putString("tilted back", " driving forward");
            }
            else{
                // m_robotDrive.drive(0, 0);
                SmartDashboard.putString("balanced", "stopping");
            }
        }
        button11State = m_stick.getRawButton(robotConstants.SMART_ORIENT_BUTTON);
        if (button11State){
            
        }
    }
}