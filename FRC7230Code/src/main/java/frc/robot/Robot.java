// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.Autonomous;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.RunMechanisms;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
  private double angleDeviation = Limelight.targetX;
  public double distance = 0;

  private String armState;

  private String armMode;
  private String clawState;
  
  private boolean midPosition;
  private boolean coneLoaded;
  private boolean autonConePickup;

  private RunMechanisms runMechanisms = Mechanisms.runMechanisms;
  private DriveTrain driveTrain = new DriveTrain(Mechanisms.driveSubsystem, runMechanisms, Mechanisms.mechanismsJoystick, Mechanisms.driveJoystick);
  private Autonomous auton = new Autonomous();
  private final SendableChooser<String> position_chooser = new SendableChooser<>();
  private final SendableChooser<String> preload_chooser = new SendableChooser<>();
  private final SendableChooser<String> autonPickup_chooser = new SendableChooser<>();


  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Drive Speed Limit", driveTrainConstants.limitX);
    SmartDashboard.putNumber("Turn Speed Limit", driveTrainConstants.limitY);


    position_chooser.setDefaultOption("Middle", "Middle");
    position_chooser.addOption("Side", "Side");
    SmartDashboard.putData("Autonomous choice", position_chooser);
    preload_chooser.setDefaultOption("Cone Preload", "Cone Preload");
    preload_chooser.addOption("Cube Preload", "Cube Preload");
    SmartDashboard.putData("Preload choice", preload_chooser);
    autonPickup_chooser.setDefaultOption("Cone", "Cone");
    autonPickup_chooser.addOption("Cube", "Cube");
    SmartDashboard.putData("Auton Pickup choice", autonPickup_chooser);
    
    Mechanisms.armPID.setP(driveTrainConstants.kP);
    Mechanisms.armPID.setI(driveTrainConstants.kI);
    Mechanisms.armPID.setD(driveTrainConstants.kD);
    Mechanisms.armPID.setFF(driveTrainConstants.kFF);
    
    
   
    // SmartDashboard.putNumber("Distance: ", distance);
    
   
  }

  @Override
  public void robotPeriodic() {
    if (driveTrain.manualLayout){
      armMode = "Manual";
    }
    else{
      armMode = "Smart";
    }
    if (Mechanisms.armSolenoid.get()){
      armState = "Extended";
    }
    else{
      armState = "Retracted";
    }
    if (Mechanisms.clawSolenoid.get()){
      clawState = "Open";
    }
    else{
      clawState = "Closed";
    }

    SmartDashboard.putString("Claw Extension State", clawState);
    SmartDashboard.putString("Arm Extension State", armState);
    SmartDashboard.putString("Selected Object", Limelight.targetName);
    SmartDashboard.putString("Arm Mode", armMode);
    SmartDashboard.putString("Arm Angle", runMechanisms.armAngle);
    SmartDashboard.putNumber("Driving Speed", driveTrain.speedX);
    SmartDashboard.putNumber("Angle to Target: ", angleDeviation);
    driveTrain.speedLimitChangeX = SmartDashboard.getNumber("Drive Speed Limit", driveTrainConstants.limitX);
    driveTrain.speedLimitChangeY = SmartDashboard.getNumber("Turn Speed Limit", driveTrainConstants.limitY);
    String positionSelected = position_chooser.getSelected();
    switch (positionSelected) {
      case "Middle":
        midPosition = true;
        break;
      case "Side":
        midPosition = false;
        break;
    }

    String preloadSelected = preload_chooser.getSelected();
    switch (preloadSelected) {
      case "Cone Preload":
        coneLoaded = true;
        break;
      case "Cube Preload":
        coneLoaded = false;
        break;
    }

    String autonPickupSelected = autonPickup_chooser.getSelected();
    switch (autonPickupSelected) {
      case "Cone":
        autonConePickup = true;
        break;
      case "Cube":
        autonConePickup = false;
        break;
    }
  }

  @Override
  public void autonomousInit() {
    if (autonConePickup){
      Limelight.coneTarget = true;
    }       
    else{
      Limelight.coneTarget = false;
    }
    Limelight.updateTarget();
    auton.init();
  }

  @Override
  public void autonomousPeriodic() {
    auton.execute(midPosition, coneLoaded);
  }

  @Override
  public void teleopInit() {
    Mechanisms.gyro.calibrate();
    Mechanisms.driveSubsystem.resetEncoders();
    DriveTrain.invertAxis = -1; // Inverted axis in Autonomous, swapping front and back. Swapping them for the driver.
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.drive(false);
    runMechanisms.rotateArmToAngle();
    runMechanisms.toggleClaw(false);
    
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
