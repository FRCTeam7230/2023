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

        SmartDashboard.putNumber("Target X-coordinates", distanceToTarget);
        SmartDashboard.putNumber("Distance to target", targetAngleX);

        System.out.println(distanceToTarget);
        System.out.println(targetAngleX);

    }

    public static double getTargetAngleX() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        
        double targetX = tx.getDouble(0.0);
        targetAngleX = targetX;
        return targetAngleX;
    }

    public static double getDistanceToMiddleTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 38.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double middleNodeTapeHeight = 24.125;
        

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceToTargetInches = (middleNodeTapeHeight - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        distanceToTarget = distanceToTargetInches;
        return distanceToTarget;
    }

    public static double getDistanceToHighTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double highNodeTapeHeight = 41.875;
        

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceToTargetInches = (highNodeTapeHeight - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        distanceToTarget = distanceToTargetInches;
        return distanceToTarget;
    }
    // public static double getAngleToTarget(){
        
    //     return double me;
    // }
}
