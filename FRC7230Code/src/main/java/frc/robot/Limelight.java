package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { 

    public static double targetAngleX;

    public static double getTargetAngleX() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        
        double targetX = tx.getDouble(0.0);
        targetAngleX = targetX;
        return targetAngleX;
    }
}
