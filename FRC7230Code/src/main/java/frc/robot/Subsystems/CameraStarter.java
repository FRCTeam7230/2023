package frc.robot.Subsystems;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import org.opencv.core.Mat;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import frc.robot.Mechanisms;
public class CameraStarter { 
    public static void runCamera(int width, int height, boolean red){
        new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture(0);
            camera.setResolution(width, height);
            camera.setVideoMode(PixelFormat.kYUYV, width, height, 10);
            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Blur", width, height);
            CvSource outputStream2 = CameraServer.putVideo("Target", width, height);
            Mat source = new Mat();
            while(!Thread.interrupted()) {
              if (cvSink.grabFrame(source) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                continue;
              }
              // SmartDashboard.putString("temp", "1");
              // Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
              // outputStream.putFrame(output);
              String tempColor;
              if (red) {
                tempColor = "red";
              }
              else {
                tempColor = "blue";
              }
                // System.out.println(red);
                // Mat processed = Mechanisms.vision.process(source, tempColor);
              
              // else {
              //   Mat processed = vision.process(source, "blue");
              // }
              outputStream.putFrame(source);
              // outputStream2.putFrame(processed);
            }
        }).start();
    }
}
