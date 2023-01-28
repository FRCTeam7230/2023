// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.robotConstants;
import frc.robot.Subsystems.Autonomous;
import frc.robot.Subsystems.CameraStarter;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.RunMechanisms;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

  private DriveTrain driveTrain = new DriveTrain(Mechanisms.driveSubsystem, Mechanisms.driveJoystick);
  private Autonomous auton = new Autonomous();
  private RunMechanisms runMechanisms = new RunMechanisms();
  private boolean red = true;   
  private final SendableChooser<String> color_chooser = new SendableChooser<>();
  @Override
  public void robotInit() {
    CameraStarter.runCamera(robotConstants.cameraWidth, robotConstants.cameraLength, red);
    // color_chooser.setDefaultOption("Red", "red");
    color_chooser.setDefaultOption("Blue", "blue");
    // color_chooser.addOption("Blue", "blue");
    color_chooser.addOption("Red", "red");
    SmartDashboard.putData("Color choice", color_chooser);
  }

  @Override
  public void robotPeriodic() {
    String colorSelected = color_chooser.getSelected();
    switch (colorSelected) {
      case "red":
        red = true;
        break;
      case "blue":
        red = false;
        break;
    }    

  }

  @Override
  public void autonomousInit() {
    auton.init();
  }

  @Override
  public void autonomousPeriodic() {
    auton.execute();
  }

  @Override
  public void teleopInit() {
    Mechanisms.gyro.calibrate();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.drive(false);
    runMechanisms.runShotAndIntake(robotConstants.SHOOT_BUTTON,robotConstants.INTAKE_BUTTON, robotConstants.INTAKE_MOTOR_BUTTON, robotConstants.shootingPower, true);
    runMechanisms.runClimber(robotConstants.CLIMBER_UP_BUTTON, robotConstants.CLIMBER_DOWN_BUTTON);

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
