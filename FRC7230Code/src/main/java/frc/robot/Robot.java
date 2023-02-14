// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.robotConstants;
import frc.robot.Subsystems.Autonomous;
//import frc.robot.Subsystems.CameraStarter;
import frc.robot.Limelight;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.RunMechanisms;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {

  private DriveTrain driveTrain = new DriveTrain(Mechanisms.driveSubsystem, Mechanisms.driveJoystick);
  private Autonomous auton = new Autonomous();
  private RunMechanisms runMechanisms = new RunMechanisms();
  private boolean red = true;  
  // private boolean cube = true; // selected cube figure as a target
  // private boolean scoringMode = true; 
  private final SendableChooser<String> color_chooser = new SendableChooser<>();
  private final SendableChooser<String> figure_chooser = new SendableChooser<>();
  private final SendableChooser<String> vision_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    //CameraStarter.runCamera(robotConstants.cameraWidth, robotConstants.cameraLength, red);
    // color_chooser.setDefaultOption("Red", "red");
    color_chooser.setDefaultOption("Blue", "blue");
    // color_chooser.addOption("Blue", "blue");
    color_chooser.addOption("Red", "red");
    SmartDashboard.putData("Color choice", color_chooser);

    figure_chooser.setDefaultOption("Cube", "cube");
    figure_chooser.addOption("Cone", "cone");
    SmartDashboard.putData("Figure choice", figure_chooser);

    vision_chooser.setDefaultOption("Scoring", "Scoring");
    vision_chooser.addOption("Pickup", "Pickup");
    SmartDashboard.putData("Vision Choice", vision_chooser);
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

    // String visionModeSelected = vision_chooser.getSelected();
    // switch (visionModeSelected) {
    //   case "scoring":
    //     scoringMode = true;
    //     break;
    //   case "pickup":
    //     scoringMode = false;
    //     break;
    // }

    // String figureSelected = figure_chooser.getSelected();
    // switch (figureSelected) {
    //   case "cube":
    //     cube = true;
    //     break;
    //   case "cone":
    //     cube = false;
    //     break;
    // }    

    // Limelight.setTarget(scoringMode, cube);
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
