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
  boolean state = true;
  String state1 = String.valueOf(state);
  double angleDeviation = 0;
  public double distance = 0;
  boolean armExtensionState = true;
  boolean manualSmart = true;
  boolean cubeCone = true;
  String armExtensionState1 = String.valueOf(armExtensionState);
  private DriveTrain driveTrain = new DriveTrain(Mechanisms.driveSubsystem, Mechanisms.driveJoystick);
  private Autonomous auton = new Autonomous();
  private RunMechanisms runMechanisms = new RunMechanisms();
  private boolean midPosition;
  private final SendableChooser<String> position_chooser = new SendableChooser<>();
  boolean cone = true;   
  private final SendableChooser<String> gamepiece_chooser = new SendableChooser<>();


  @Override
  public void robotInit() {
    position_chooser.setDefaultOption("Middle", "Middle");
    position_chooser.addOption("Side", "Side");
    SmartDashboard.putData("Position choice", position_chooser);
    //CameraStarter.runCamera(robotConstants.cameraWidth, robotConstants.cameraLength, cone);
    //color_chooser.setDefaultOption("Red", "red");
    gamepiece_chooser.setDefaultOption("Cone", "cone");
    //color_chooser.addOption("Blue", "blue");
    gamepiece_chooser.addOption("Cube", "cube");
    SmartDashboard.putData("Game piece choice: ",gamepiece_chooser);
    SmartDashboard.putString("Arm Motor Position State: ",state1);
    SmartDashboard.putNumber("Angle Deviation: ",angleDeviation);
    SmartDashboard.putNumber("Distance: ",distance);
    SmartDashboard.putString("Arm Extension State: ",armExtensionState1);
    if(manualSmart){
      SmartDashboard.putString("Mode: ","Smart");
    }
    else{
      SmartDashboard.putString("Mode: ","Manual");
    }
    if(cubeCone){
      SmartDashboard.putString("Game piece chosen: ","Cube");
    }
    else{
      SmartDashboard.putString("Game piece chosen: ","Cone");
    }
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
    String gamePieceSelected = gamepiece_chooser.getSelected();
    switch (gamePieceSelected) {
      case "cone":
        cone = true;
        
        break;
      case "cube":
        cone = false;
        break;
    }
    double armMotorPos = RunMechanisms.PIDEncoder.getPosition();   
    SmartDashboard.putNumber("Arm Motor Position", armMotorPos);
    double driveSpeed = RunMechanisms.PIDEncoder.getVelocity();
    SmartDashboard.putNumber("Driving Speed",driveSpeed);
    SmartDashboard.putNumber("Drive Speed X Limit",driveTrainConstants.limitX);
    SmartDashboard.putNumber("Drive Speed Y Limit",driveTrainConstants.limitY);
    SmartDashboard.getNumber("Drive Speed X",driveTrainConstants.limitX );
    SmartDashboard.getNumber("Drive Speed Y",driveTrainConstants.limitY );
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
