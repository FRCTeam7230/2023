// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.Autonomous;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.RunMechanisms;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



public class Robot extends TimedRobot {

  private RunMechanisms runMechanisms = Mechanisms.runMechanisms;
  private DriveTrain driveTrain = new DriveTrain(Mechanisms.driveSubsystem, runMechanisms, Mechanisms.driveJoystick);
  private Autonomous auton = new Autonomous();
  private boolean midPosition;
  
  private final SendableChooser<String> position_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    position_chooser.setDefaultOption("Middle", "Middle");
    position_chooser.addOption("Side", "Side");
    SmartDashboard.putData("Autonomous choice", position_chooser);
    SmartDashboard.putString("Selected Object", Limelight.targetName);
    Mechanisms.armPID.setP(driveTrainConstants.kP);
    Mechanisms.armPID.setI(driveTrainConstants.kI);
    Mechanisms.armPID.setD(driveTrainConstants.kD);
    Mechanisms.armPID.setFF(driveTrainConstants.kFF);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.updateValues();
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
    Mechanisms.driveSubsystem.resetEncoders();
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
