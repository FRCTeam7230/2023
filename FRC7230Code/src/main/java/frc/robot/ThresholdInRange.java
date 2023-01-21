
// package frc.robot;
// import org.opencv.core.Point;
// import org.opencv.imgproc.Imgproc;

// import java.util.List;

// import org.opencv.core.Core;
// import org.opencv.core.Mat;
// import org.opencv.core.Scalar;
// import java.util.ArrayList;

// public class ThresholdInRange {   
//     private static int screenCenterX = 80;
//     //screen size: x= 634, y =  480
    
//     //measurement:
//     private static double focalLength = 320.8; //focal length in pixels
//     private static double ballRadius = 12.5;
//     private static double distanceCameraToBall = 0;
    
//     private static double depth = ballRadius;
//     private static int robotDepth = 13;
//     private static double cameraAngle = 55.0;//change this to another angle from flour
//     public static double ballDistance;
//     public static double ballAngleX, ballAngleY;
    
//     // private VideoCapture cap;
//     // private Mat matFrame = new Mat();
//     // private JFrame frame;
//     // private JLabel imgCaptureLabel;
//     // private JLabel imgDetectionLabel;
//     // private CaptureTask captureTask;

//     private static Mat thresh = new Mat();
//     private static Mat frameHSV = new Mat();
//     private static Mat circles = new Mat();
    
//     public Mat process(Mat frame, String color) {
//             // Mat frameHSV = new Mat();  
//         Imgproc.cvtColor(frame, frameHSV, Imgproc.COLOR_BGR2HSV);
//         // Mat thresh = new Mat();
        
//         //red color - need to change
//         if (color == "red"){
//             Core.inRange(frameHSV, new Scalar(0, 80, 40),
//                 new Scalar(15, 255, 255), thresh);
//         }
//         else{
//             //Old Values for blue:
//             // Core.inRange(frameHSV, new Scalar(95, 50, 0),
//             //         new Scalar(110, 255, 255), thresh);
//             // New values for Blue: Low HSV; 90, 110, 110 High HSV; 110, 255, 255
//             Core.inRange(frameHSV, new Scalar(90, 110, 110), //100,
//                     new Scalar(120, 200, 255), thresh);
           
//         }
    
//     List<Mat> frames = new ArrayList<Mat>(); //new List<Mat>();
//         Core.split(thresh, frames);
//         Mat gray = frames.get(0);
    
//         // Imgproc.putText(frame, ".", new Point(screenCenterX, screenCenterY), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 3);	
//         Imgproc.medianBlur(gray, gray, 5);
//         // Mat circles = new Mat();    	        	  
        
//         Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 2.0,
//                 2*(double)gray.rows(), // change this value to detect circles with different distances to each other
//                 50.0, 30.0, 10, 0); // change the last two parameters
//                         // (m2in_radius & max_radius) to detect larger circles - need to change min radius to normal values
    
//         for (int x = 0; x < circles.cols(); x++) {
    
//         double[] c = circles.get(0, x);
                    
//             int cX = (int) Math.round(c[0]/5 - 1)*5; //coordinatesX and coordinatesY
//             int cY = (int) Math.round(c[1]/5 - 1)*5;
//             int radius = (int) Math.round(c[2]);
                    
//             // String coordinateXY = cX + "," + cY;
                    
//                     // circle center
//             // Imgproc.circle(frame, new Point(cX, cY), 1, new Scalar(0,255,100), 3, 8, 0);
//                     // circle outline
//             Imgproc.circle(frame, new Point(cX, cY), radius, new Scalar(255,0,255), 2, 8, 0);
                    
//             // Imgproc.putText(frame, coordinateXY, new Point(cX, cY), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 111), 2);
//                 //distance from camera to ball
//                 distanceCameraToBall = Math.round(focalLength*ballRadius/radius) - depth;
//                     //distance from robot to ball                      
//                 ballDistance = (double) Math.round(distanceCameraToBall*Math.sin(Math.toRadians(cameraAngle))/2)*2- robotDepth;
    
//                 //helpful variables:
//                 int kat1 = cX-screenCenterX;
//                 // int kat2 = cY-screenCenterY;
            
//                 ballAngleX = (double) Math.round(Math.toDegrees(Math.atan(ballRadius*kat1/(radius*ballDistance)))/2)*2;
//                 // ballAngleY = (double) Math.round(Math.toDegrees(Math.atan(ballRadius*kat2/(radius*ballDistance)))/2)*2;
                
//                 // String stringBallAngleX = ballAngleX + " X";
//                 // String stringBallAngleY =  ballAngleY + " Y "; 
                
//                 // Imgproc.putText(frame, Dsize, new Point(10, 20), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(255, 255, 0), 1);	
//                 // Imgproc.putText(frame, stringBallAngleX, new Point(20, 100), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);	
//                 // Imgproc.putText(frame, stringBallAngleY, new Point(20, 150), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);
//             }
//         return thresh;
//     }
   
//     public double getDistance(){
//         return ballDistance;
//     }
//     public double getAngleX() {
//         return ballAngleX;
//     }   
// }