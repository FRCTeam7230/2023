package frc.robot;

import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.RunMechanisms;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Mechanisms {
    public static CANSparkMax armMotor = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
    public static SparkMaxPIDController armPID = armMotor.getPIDController();
    public static Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    public static Solenoid armSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    public static Solenoid clawLeftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    public static Solenoid clawRightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static Joystick driveJoystick = new Joystick(0);
    public static Joystick mechanismsJoystick = new Joystick(1);
    public static RunMechanisms runMechanisms = new RunMechanisms();
    public static AHRS gyro = new AHRS();
    
}
