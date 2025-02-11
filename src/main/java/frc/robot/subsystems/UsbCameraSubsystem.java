// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj2.command.Command;
import org.opencv.core.Core;

import org.opencv.core.Rect;

/*
 * Susbystem for the usb camera on the robot that puts video feed on the cameraserver class or however that works. 
 * can apply various openCV imgproc effects to the video feed such as gaussian blur or rawing overlays. 
 * Processed video feed is available in camera server named "Processed Feed"
 * Raw Video feed should be available in the defaulst usbcamera server.
 * Also includes command to toggle the camera processing effects called "toggle overlay"
 */
public class UsbCameraSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //vision thread processes vision
      private Thread visionThread;
      //overlay on top of video feed toggle.
      private boolean overlay = true;
      private boolean flip = true;
      //most recent frame from vision
      private Mat latestMat;

      private boolean cameraStarted = false;
      
      //Constructor that creates the image processing thread.
  public UsbCameraSubsystem() {
//StartCamera(0);
cameraStarted = false;
  }
  //A few commands for the blur effect just for testing purposes.

/*
 * This is the main method for applying video effects.
 * Alter the inputMat in any way you need, after this method runs the mat will then be uploaded to the cameraServer. 
 */
private void processVideoFeed(Mat inputMat)
{
  Point target = VisionSubsystem.getReeftargetCenter();
  Rect targetBox = VisionSubsystem.locateReefPipeTarget();
  if(target.x != 0&&target.y != 0)
  {
    Imgproc.putText(inputMat, "TARGET", new Point(target.x-25,target.y+8), 0, 0.4, new Scalar(0,0,0,0));

    Imgproc.circle(inputMat, target,30, new Scalar(0,0,0),3); 
    Scalar color =  new Scalar(225,20,250);
    if(VisionSubsystem.isReady())
    {
      color = new Scalar(0,255,0);
    }
    Imgproc.rectangle(inputMat, targetBox,color,2);




  } 
  //reef pipe visualizer
  //Imgproc.rectangle( inputMat, new Point(280, 180), new Point(360, 480), new Scalar(225, 20, 250), 5);
  Imgproc.putText(inputMat, "Processed Camera Feed", new Point(20,50), 0, 1, new Scalar(0,0,0),3);


}

//toggles overlay such as reef pipe detection indicators
public Command toggleOverlay()
{
  return   runOnce(()->
  {overlay = !overlay;}
  );
}
public Command toggleFlip()
{
  return runOnce(()->
  {flip = !flip;}
  );
}
// returns latest unprocessed video frame
public Mat getLatestFrame()
{
  return latestMat;
}
//begins camera on port number
public Command StartCameraFeed(int port)
{
  return runOnce(()->
  {
    if(!cameraStarted)
    {
      StartCamera(port);
    }
    cameraStarted = true;
  }
  );

}

// starts camera in its own thread so that it doesnt interfere with other robot processes
private void StartCamera(int dev)
{
  visionThread =
  new Thread(

      () -> {
        UsbCamera camera = CameraServer.startAutomaticCapture(dev);

        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Processed Feed", 640, 480);

        // Mats are very memory expensive. Lets reuse this Mat.
        Mat mat = new Mat();

        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
          long startTime = System.currentTimeMillis();

          // Tell the CvSink to grab a frame from the camera and put it
          // in the source mat.  If there is an error notify the output.
          if (cvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            // skip the rest of the current iteration
            continue;
          }
          // Apply image processing to image
          if(flip)
          Core.flip(mat,mat,0);

          if(overlay)
          {
            processVideoFeed(mat);
                SmartDashboard.putNumber("EstimatedDistanceFromPipe:",0.01*(int)(VisionSubsystem.getReefPipeDistance()*100));
                SmartDashboard.putNumber("Estimated horizontal offset",0.01*(int)(VisionSubsystem.getReefTargetOffset()*100));
          }
        

          // Give the output stream a new image to display 

          outputStream.putFrame(mat);
          latestMat = mat;
          long time = System.currentTimeMillis()-startTime;
          SmartDashboard.putNumber("Vision FPS: ",((int)1000.00/time));
        }
      });
visionThread.setDaemon(true);
visionThread.start();
}

}
