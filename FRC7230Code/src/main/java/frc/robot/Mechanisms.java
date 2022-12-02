package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Subsystems.DriveSubsystem;
public class Mechanisms {
    public static CANSparkMax shooterMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax conveyorMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    public static VictorSPX intakeMotor = new VictorSPX(7);
    public static CANSparkMax climberMotor = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);
    public static Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    public static Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    public static Solenoid climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    public static ThresholdInRange vision = new ThresholdInRange();
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static Joystick joystick = new Joystick(0);
}
