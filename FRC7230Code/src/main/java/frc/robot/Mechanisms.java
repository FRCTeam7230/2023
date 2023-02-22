package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Subsystems.DriveSubsystem;
public class Mechanisms {
    public static CANSparkMax armMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    public static Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    // public static Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    public static Solenoid climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    // public static ThresholdInRange vision = new ThresholdInRange();
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static Joystick driveJoystick = new Joystick(0);
    public static Joystick mechanismsJoystick = driveJoystick;
    public static AHRS gyro = new AHRS();//SPI.Port.kMXP);

    public static Solenoid armSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,1);
    public static Solenoid clawRightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,2);
    public static Solenoid clawLeftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,3);
    //check id later
}
