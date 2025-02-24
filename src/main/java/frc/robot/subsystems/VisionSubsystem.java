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
//import org.opencv.core.Scalar;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static UsbCameraSubsystem cameraSubsystem;
  private static final ReefDetectionPipeline reefDetection = new ReefDetectionPipeline();
  //This value is for the reeef pipe detection of width. It is the amount of extra pixels blocked from the camera view above the second-highest pipe.
  
  //Adjust this if the robot does not detect a pipe.
  //private static int safetyPixels = 20;

  private static boolean isTargetValid;

  private static double cameraOffsetInches = 0;
  private static double lastPixelValue;

  public static double AcceptableError = 0.5; // inches

  public static Rect lastRect;



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
  public static void RefreshVisionData()
  {
    lastRect = locateReefPipeTarget();
  }


  //returns bounding rect of reef pipe target
  //this works by first running the vision pipeline to find all purple contours in frame. Then it applies some filtering by slicing o all the camera feed except
  // for the top of the tallest contour. This allows us to find the width of the closest reef pipe without having to worry about interference/ other pipes in frame. 
  
  public static Rect locateReefPipeTarget()
  {
  Mat latest = getLatestFrame();

  if(latest == null) {
    isTargetValid = false;
    return new Rect(0,0,0,0);
  }

  // reefDetection.process(latest);
  ArrayList<MatOfPoint> contoursResult = reefDetection.filterContoursOutput();
  //find largest contours
  Rect largestBox = new Rect(0,0,0,0);
  Rect secondLargest = new Rect(0,0,0,0);
  for(int i = 0; i< contoursResult.size(); i++)
  {
    Rect bound = Imgproc.boundingRect(contoursResult.get(i));
    //uses height of rect as comparison metric
    double height = bound.br().y-bound.tl().y;
    if(height>largestBox.br().y-largestBox.tl().y)
    {
      secondLargest = largestBox;
      largestBox = bound;
    }
  }

  if(Math.abs(largestBox.height-secondLargest.height)<50)
  {
    if(largestBox.width<secondLargest.width)
    {
      Rect temp = largestBox;
      largestBox = secondLargest;
      secondLargest = temp;
    }
  }
  double br = largestBox.br().y;
  /* 
  //chop off bottom of frame
  if(secondLargest.area()>0)
  {
    Mat copied = new Mat();
    latest.copyTo(copied);
    int h = (int)(480-secondLargest.tl().y)-safetyPixels;
    //inefficient chopping code - draws a really thick line across bottom of frame. 
    Imgproc.rectangle(copied, new Rect(new Point(0,480),new Point(640,479)), new Scalar(0,0,0),h*2);
  
    //run second vision pass to get more accurate results. 
    reefDetection.process(copied);
    ArrayList<MatOfPoint> newContoursResult = reefDetection.filterContoursOutput();
    //find largest contour
    largestBox = new Rect(0,0,0,0);
    for(int i = 0; i< newContoursResult.size(); i++)
    {
      Rect bound = Imgproc.boundingRect(newContoursResult.get(i));
      
      double area = bound.area();
      if(area>largestBox.area())
      {
        largestBox = bound;
      }
    }
  }
    */
  Rect result = new Rect(largestBox.tl(),new Point(largestBox.br().x,br));
  isTargetValid = true;
  return result;
  }
// returns the pixel width of reef pipe target
  public static double getPixelsWidth()
{
  double res =  (lastRect.width);
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
  if(pixels == 0)
  {
    pixels = lastPixelValue;
  }
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
  return getCenter(lastRect);
}
/**
 * 
 * @return pixels from camera center to reef pipe center horizontally. Positive means pipe is to the right
 */
public static double getPipePixelOffset()
{
  double pix = getReeftargetCenter().x;
  double offset = pix-320;
  return offset;
}
public static double getReefTargetOffset()
{
  double pixelOffset = getPipePixelOffset();
  double pixelsScale = getPixelsWidth()/1.25;
  if(pixelsScale==0)
  {
    pixelsScale = lastPixelValue;
  }
  else
  {
    lastPixelValue = pixelsScale;
  }
  return pixelOffset/pixelsScale-cameraOffsetInches;
}

  public static boolean isTargetValid()
  {
    return isTargetValid;
  }

  public static  boolean isReady()
  {
    if(Math.abs(getReefTargetOffset())<AcceptableError)
    {
      return true;
    }
    return false;
  }

  
}
