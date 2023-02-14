package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { 

    public static double targetX, targetY;
    public static double targetArea;

    private static double cubeAreaPrc = 10.5; //percentage of cube on the screen to be able to get the piece 
    private static double coneAreaPrc = 10.0; //percentage of cube on the screen to be able to get the piece 
    private static double tapeAreaPrc = 0.09; //percentage of cube on the screen to be able to score game piece 
    private static double objAreaRequired = 0.0;

    public  static boolean moveToTarget = true;

    public static void updateData(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        
        //read values periodically
        targetX = tx.getDouble(0.0);
        targetY = ty.getDouble(0.0);
        targetArea = ta.getDouble(0.0);
    }

    public static void setTarget(int targetNumber) { // targetNumber: 0 -tape, 1 - cube, 2 - cone
        if(targetNumber == 0) {
            objAreaRequired = tapeAreaPrc;
        }
        else if (targetNumber == 1){
            objAreaRequired = cubeAreaPrc;
        }
        else {
            objAreaRequired = coneAreaPrc;
        }
    }

    public static double getTargetAngleX() {
        updateData();
        return targetX;
    }

    public static boolean procedMoving() {
        updateData();
        System.out.println(targetArea);

        if (targetArea >= objAreaRequired) {
            moveToTarget = false;
        }
        // else {
        //     moveToTarget = true;
        // }
        return moveToTarget; 
    }

    // public static double getDistanceToMiddleTarget() {
    //     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //     NetworkTableEntry ty = table.getEntry("ty");
    //     double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    //     // how many degrees back is your limelight rotated from perfectly vertical?
    //     double limelightMountAngleDegrees = 38.0;

    //     // distance from the center of the Limelight lens to the floor
    //     double limelightLensHeightInches = 20.0;

    //     // distance from the target to the floor
    //     double middleNodeTapeHeight = 24.125;
        

    //     double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //     //calculate distance
    //     double distanceToTargetInches = (middleNodeTapeHeight - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    //     distanceToTarget = distanceToTargetInches;
    //     return distanceToTarget;
    // }

    // public static double getDistanceToHighTarget() {
    //     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //     NetworkTableEntry ty = table.getEntry("ty");
    //     double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    //     // how many degrees back is your limelight rotated from perfectly vertical?
    //     double limelightMountAngleDegrees = 25.0;

    //     // distance from the center of the Limelight lens to the floor
    //     double limelightLensHeightInches = 20.0;

    //     // distance from the target to the floor
    //     double highNodeTapeHeight = 41.875;
        

    //     double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //     //calculate distance
    //     double distanceToTargetInches = (highNodeTapeHeight - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    //     distanceToTarget = distanceToTargetInches;
    //     return distanceToTarget;
    // }
}
