package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight { 

    public static double targetAngleX;
    
        public static double targetX, targetY;
        public static double targetArea;
        public static boolean coneTarget = true; // true if cone is target, false if cube
        public static String targetName = "cone";
        
        // private static double cubeAreaPrc = 10.5; //percentage of cube on the screen to be able to get the piece 
        // private static double coneAreaPrc = 10.0; //percentage of cube on the screen to be able to get the piece 
        // private static double tapeAreaPrc = 0.09; //percentage of cube on the screen to be able to score game piece 
        // private static double objAreaRequired = 0.0;

        private static double objAreaMargin = 0.15;
        private static double objAreaError;
        public  static boolean continueMoving = true;
    
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
    
        // public static void setTarget(int targetNumber) { // targetNumber: 0 -tape, 1 - cube, 2 - cone
        //     if(targetNumber == 0) {
        //         objAreaRequired = tapeAreaPrc;
        //     }
        //     else if (targetNumber == 1){
        //         objAreaRequired = cubeAreaPrc;
        //     }
        //     else {
        //         objAreaRequired = coneAreaPrc;
        //     }
        // }
    
        public static double getTargetAngleX() {
            updateData();
            return targetX;
        }
    
        public static boolean procedMoving(double objAreaRequired) {
            updateData();
            objAreaError = objAreaRequired - targetArea;
            

            if (Math.abs(objAreaError) <= objAreaMargin) {
                continueMoving = false;
            }
            // else {
            //     moveToTarget = true;
            // }
            return continueMoving; 
        }
}
