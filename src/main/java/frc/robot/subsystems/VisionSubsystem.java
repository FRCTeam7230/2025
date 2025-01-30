// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static UsbCameraSubsystem cameraSubsystem;
  private static final ReefDetectionPipeline reefDetection = new ReefDetectionPipeline();
  private static double lastRangeValue = 0;
//initializes the camera feed to the vision system
  public VisionSubsystem(UsbCameraSubsystem camSys)
  {
    cameraSubsystem = camSys;
  }

  // returns latest frame from visionsubsystem
  private static Mat getLatestFrame()
  {
    return cameraSubsystem.getLatestFrame();
  }
  //returns bounding rect of reef pipe target
  public static Rect locateReefPipeTarget()
  {
  Mat latest = getLatestFrame();
  if(latest == null) return new Rect(0,0,0,0);
  reefDetection.process(getLatestFrame());
  ArrayList<MatOfPoint> contoursResult = reefDetection.filterContoursOutput();
  //find largest contour
  Rect largestBox = new Rect(0,0,0,0);
  for(int i = 0; i< contoursResult.size(); i++)
  {
    Rect bound = Imgproc.boundingRect(contoursResult.get(i));
    double area = bound.area();
    if(area>largestBox.area())
    {
      largestBox = bound;
    }
  }
  return largestBox;
  }
// returns the pixel width of reef pipe target
  public static double getPixelsWidth()
{
  double res =  (VisionSubsystem.locateReefPipeTarget().width*lastRangeValue)/2;
  if(Math.abs(res-lastRangeValue)>50)
  {
    res = lastRangeValue;
  }
  lastRangeValue = VisionSubsystem.locateReefPipeTarget().width;
  return res;
}
// converts pixels from camera of reef pipe to inches away from camera. 
private static double convertReefPipeToInches(double pixels)
{
  //function calculated by using the following dataset:
  //all values computed using a 1 1/4 inch PVC Pipe
  /*
  24 inches - 63 pixels 
  23 inches - 64 pixels
  22 inches - 66 pixels
  21 inches - 67 pixels
  20 inches - 72 pixels
  19 inches - 76 pixels
  18 inches - 80 pixels
  17 inches - 82 pixels
  16 inches - 85 pixels
  15 inches - 90 pixels
  14 inches - 93 pixels
  13 inches - 100 pixels
  12 inches - 118 pixels
  11 inches - 122 pixels
  10 inches - 130 pixels
  9 inches - 138 pixels
  8 inches - 150 pixels
  7 inches - 162 pixels
  6 inches - 175 pixels
  5 inches - 205 pixels
  4 inches - 270 pixels
  3 inches - 340 pixels
  2 inches - 450 pixels
  1 inches - 640 pixels
   */
  
  return 1584.78/pixels-2.186;

}

//Method intended to be used to get the distance from the reef pipe target to the usb camera.
//Error ~= +- 1 inch depending on noise, interfering objects
//Use With Caution - this is only accurate when the camera and pipe are level and the rectangular bounding box as display on the camera feed is correctly placed by the pipeline. 
//Also only calibrated for when the pipe is directly in front of camera - align robot laterally before using this method.

public static double getReefPipeDistance()
{
  return convertReefPipeToInches(getPixelsWidth());
}
//helper method that calculates center of OpenCV rect
private static Point getCenter(Rect rect)
{
  Point p1 = rect.tl();
  Point p2 = rect.br();
  return new Point((p1.x+p2.x)/2,(p1.y+p2.y)/2);
}
//gets  pixel location on camera of reef pipe target

public static Point getReeftargetCenter()
{
  return getCenter(VisionSubsystem.locateReefPipeTarget());
}


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
