// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.Autonomous;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.RunMechanisms;
import frc.robot.Constants.*;
import frc.robot.Subsystems.CameraStarter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
  private String insideVisionMargin;
  private String driveAxisIndicator;
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
    CameraStarter.runCamera(160, 120);
    position_chooser.setDefaultOption("Middle", "Middle");
    position_chooser.addOption("Side", "Side");
    SmartDashboard.putData("Autonomous choice", position_chooser);
    preload_chooser.setDefaultOption("Cone Preload", "Cone Preload");
    preload_chooser.addOption("Cube Preload", "Cube Preload");
    SmartDashboard.putData("Preload choice", preload_chooser);
    autonPickup_chooser.setDefaultOption("Cone", "Cone");
    autonPickup_chooser.addOption("Cube", "Cube");
    SmartDashboard.putData("Auton Pickup choice", autonPickup_chooser);
    Limelight.initializeLimelightOff();

    // CHECK THIS
    Mechanisms.armMotorEncoder.setDistancePerRotation(360);
    // Mechanisms.armMotorEncoder.setPositionConversionFactor(driveTrainConstants.rotationsToDegrees);
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
    if (DriveTrain.invertAxis == 1){
      driveAxisIndicator = "Normal";
    }
    else{
      driveAxisIndicator = "Inverted";
    }
    if (driveTrain.insideVisionMargin){
      insideVisionMargin = "Centered";
    }
    else{
      insideVisionMargin = "Not Centered";
    }
    
    driveTrain.speedLimitChangeX = SmartDashboard.getNumber("Drive Speed Limit", driveTrainConstants.limitX);
    driveTrain.speedLimitChangeY = SmartDashboard.getNumber("Turn Speed Limit", driveTrainConstants.limitY);
    

    SmartDashboard.putString("Drive Axis Mode", driveAxisIndicator);
    SmartDashboard.putString("Claw Extension State", clawState);
    SmartDashboard.putString("Arm Extension State", armState);
    SmartDashboard.putString("Selected Object", Limelight.targetName);
    SmartDashboard.putString("Arm Mode", armMode);
    SmartDashboard.putString("Arm Angle", runMechanisms.armAngle);
    SmartDashboard.putNumber("Driving Speed", driveTrain.speedX);
    SmartDashboard.putString("Centered Indicator", insideVisionMargin);
   

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
    runMechanisms.toggleClaw(false);
    runMechanisms.testArm(Mechanisms.driveJoystick);
    runMechanisms.testExtension(Mechanisms.driveJoystick);
    runMechanisms.closeArm(Mechanisms.driveJoystick);
    runMechanisms.rotateArmToAngle();
    runMechanisms.checkArm();

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
/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
/* 
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
//   @Override
//   public void robotInit() {
//     // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//     // autonomous chooser on the dashboard.
//     m_robotContainer = new RobotContainer();
//   }

//   /**
//    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
//    * that you want ran during disabled, autonomous, teleoperated and test.
//    *
//    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
//    * SmartDashboard integrated updating.
//    */
//   @Override
//   public void robotPeriodic() {
//     // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
//     // commands, running already-scheduled commands, removing finished or interrupted commands,
//     // and running subsystem periodic() methods.  This must be called from the robot's periodic
//     // block in order for anything in the Command-based framework to work.
//     CommandScheduler.getInstance().run();
//   }

//   /** This function is called once each time the robot enters Disabled mode. */
//   @Override
//   public void disabledInit() {}

//   @Override
//   public void disabledPeriodic() {}

//   /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   @Override
//   public void autonomousInit() {
//     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

//     /*
//      * String autoSelected = SmartDashboard.getString("Auto Selector",
//      * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
//      * = new MyAutoCommand(); break; case "Default Auto": default:
//      * autonomousCommand = new ExampleCommand(); break; }
//      */

//     // schedule the autonomous command (example)
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.schedule();
//     }
//   }

//   /** This function is called periodically during autonomous. */
//   @Override
//   public void autonomousPeriodic() {}

//   @Override
//   public void teleopInit() {
//     // This makes sure that the autonomous stops running when
//     // teleop starts running. If you want the autonomous to
//     // continue until interrupted by another command, remove
//     // this line or comment it out.
//     if (m_autonomousCommand != null) {
//       m_autonomousCommand.cancel();
//     }
//   }

//   /** This function is called periodically during operator control. */
//   @Override
//   public void teleopPeriodic() {}

//   @Override
//   public void testInit() {
//     // Cancels all running commands at the start of test mode.
//     CommandScheduler.getInstance().cancelAll();
//   }

//   /** This function is called periodically during test mode. */
//   @Override
//   public void testPeriodic() {}
// }
