package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { 

    public static double targetX, targetY;
    public static double targetArea;
    public static double distanceToTarget;
    public static double targetAngleX;

    private static double focalLength = 0.0; //focal length in pixels
    private static int robotDepth = 0;

    //cube:
    private static double cubeHeight = 0.0;
    private static double cubeHeightPxl = 0.0;

    //cone:
    private static double coneSide = 0.0;
    private static double coneHeight = 0.0;
    private static double coneHeightPxl = 0.0;

    public static void setUpLimelightData(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically (куда эти строчки кода?)
        double targetX = tx.getDouble(0.0);
        double targetY = ty.getDouble(0.0);
        double targetArea = ta.getDouble(0.0);
    }

    public static void getTargetPosition(boolean cube) {

        if(cube){
            distanceToTarget = Math.round(focalLength*cubeHeight/cubeHeightPxl) - cubeHeight/2 - robotDepth;
            targetAngleX = (double) Math.round(Math.toDegrees(Math.atan(cubeHeight*targetX/(cubeHeightPxl*distanceToTarget)))/2)*2;
        }
        else {
            distanceToTarget = Math.round(focalLength*coneHeight/coneHeightPxl) - coneSide/2 - robotDepth;
            targetAngleX = (double) Math.round(Math.toDegrees(Math.atan(cubeHeight*coneHeight/(coneHeightPxl*distanceToTarget)))/2)*2;
        }

        SmartDashboard.putNumber("Tardget X-coordinates", distanceToTarget);
        SmartDashboard.putNumber("Distance to target", targetAngleX);

        System.out.println(distanceToTarget);
        System.out.println(targetAngleX);

    }
}
