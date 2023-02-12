// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.Autonomous;

import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.RunMechanisms;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
public class Robot extends TimedRobot {

  private DriveTrain driveTrain = new DriveTrain(Mechanisms.driveSubsystem, Mechanisms.driveJoystick);
  private Autonomous auton = new Autonomous();
  private RunMechanisms runMechanisms = new RunMechanisms();
  private boolean midPosition;
  private final SendableChooser<String> position_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    position_chooser.setDefaultOption("Middle", "Middle");
    position_chooser.addOption("Side", "Side");
    SmartDashboard.putData("Position choice", position_chooser);
  }

  @Override
  public void robotPeriodic() {
    String positionSelected = position_chooser.getSelected();
    switch (positionSelected) {
      case "Middle":
        midPosition = true;
        break;
      case "Side":
        midPosition = false;
        break;
    }
  }

  @Override
  public void autonomousInit() {
    auton.init();
  }

  @Override
  public void autonomousPeriodic() {
    auton.execute(midPosition);
  }

  @Override
  public void teleopInit() {
    Mechanisms.gyro.calibrate();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.drive(false);
    // runmechanisms.

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
