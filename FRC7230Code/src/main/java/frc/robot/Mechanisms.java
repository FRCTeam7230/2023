package frc.robot;

import frc.robot.Constants.driveTrainConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.RunMechanisms;
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
public class Mechanisms {
    public static CANSparkMax armMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    public static Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    public static Solenoid armSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    public static Solenoid clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static Joystick driveJoystick = new Joystick(0);
    public static Joystick mechanismsJoystick = new Joystick(1);
    public static AHRS gyro = new AHRS();
    // public static final SparkMaxAbsoluteEncoder armMotorEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    public static final DutyCycleEncoder armMotorEncoder = new DutyCycleEncoder(9);
    public static final PIDController armController = new PIDController(driveTrainConstants.kP, driveTrainConstants.kI , driveTrainConstants.kD, driveTrainConstants.kFF);
    public static final DigitalInput upperLimitSwitch = new DigitalInput(8);
    public static final DigitalInput lowerLimitSwitch = new DigitalInput(7);
    public static RunMechanisms runMechanisms = new RunMechanisms();
}
