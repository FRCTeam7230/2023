package frc.robot;

import frc.robot.Constants.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { 

    public static double targetAngleX;
    
    public static double targetX, targetY, visionTargets;

    public static double targetArea;
    public static boolean coneTarget = false; // true if cone is target, false if cube
    public static String targetName = "Cube";
    public static double[] gamePieceAreas = driveTrainConstants.coneScreenAreas;
    public static double tapeScreenArea;
    
    // private static double cubeAreaPrc = 10.5; //percentage of cube on the screen to be able to get the piece 
    // private static double coneAreaPrc = 10.0; //percentage of cube on the screen to be able to get the piece 
    // private static double tapeAreaPrc = 0.09; //percentage of cube on the screen to be able to score game piece 
    // private static double objAreaRequired = 0.0;

    private static double objAreaMargin = 0.10;
    private static double objAreaError;
    public  static boolean continueMoving = false;

    public static void updateData(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        
        //read values periodically
        visionTargets = tv.getDouble(0.0);
        targetX = tx.getDouble(0.0);
        targetY = ty.getDouble(0.0);
        targetArea = ta.getDouble(0.0);
    }

    public static double[] updateTarget(){
        // Changing targets' characteristics to cones' or cubes'
        if (coneTarget) { 
            // ! check pipelines - their numbers + values + they should be upside down - both cube and cones
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2); 
            gamePieceAreas = driveTrainConstants.coneScreenAreas;

            targetName = "Cone";
        }
        else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
            gamePieceAreas = driveTrainConstants.cubeScreenAreas;

            targetName = "Cube";
        }
        return gamePieceAreas;
    }

    public static void manualLimelightOff(boolean manualLayout){
        if (manualLayout){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
        }

    }

    public static void initializeLimelightOff(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
    }

    public static boolean updateTape(boolean highTarget, double[]tapeScreenAreas){
        if(highTarget && coneTarget) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(7); 
            tapeScreenArea = driveTrainConstants.tapeScreenAreas[0];
        }
        else if(!highTarget && coneTarget){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(6); 
            //create pipeline 6 for middle target
            tapeScreenArea = driveTrainConstants.tapeScreenAreas[1];
        }
        else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
        }
        return proceedMoving(tapeScreenArea);
    }

    public static double getTargetAngleX() {
        updateData();
        return targetX;
    }

    public static double getTargetAngleY(){
        updateData();
        return targetY;
    }

    public static boolean proceedMoving(double objAreaRequired) {
        updateData();
        objAreaError = objAreaRequired - targetArea;
        // System.out.println(targetArea);
        if (Math.abs(objAreaError) <= objAreaMargin) {
            continueMoving = false;
        }
        return continueMoving; 
    }
}
