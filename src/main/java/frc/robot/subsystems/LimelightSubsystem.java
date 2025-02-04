// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private double metersToFeet = 0.3048;
  private double targetingOffsetX;
  private double targetingOffsetY;
  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {}
  /**
   * @return limelight offset in feet from primary in-view apriltag (asuming apriltag pipeline is active) [forward/backward, left/right, up/down]
   */
  public double[] getTargetOffset()
  {
    double[] limelightResults = LimelightHelpers.getTargetPose_RobotSpace("");
    double[] result = new double[3];
    result[0] = limelightResults[0]*metersToFeet+targetingOffsetX;
    //forward positive

    result[1] = limelightResults[1]*metersToFeet+targetingOffsetY;  
    //right positive

    result[2] = limelightResults[2]*metersToFeet;
    //up positive

    return result;
  }
    /**
   * @return limelight offset in degrees of rotation from primary in view apriltag (assuming apriltag pipeline is active)
   */
  public double getTargetRotation()
  {
    double[] limelightResults = LimelightHelpers.getBotPose_TargetSpace("");

    double yaw = limelightResults[4];// this is the yaw value of the robot in the targetSpace - testings needed to determine whether positive angels returned are left/right 
    return yaw;
  }
  /**
   * 
   * @param x offset in feet from apriltg of desired position -> negative is further from reef.
   * @param y offset in feet horizontally from apriltag target -> positive is offset to the Right of apriltag. 
   */
  public void setTargetingOffset(double x, double y)
  {
    targetingOffsetX = x;
    targetingOffsetY = y;
  }
  public void SetTargetLeft()
  {
    targetingOffsetY = Math.abs(targetingOffsetY)*-1;
  }
  public void SetTargetRight()
  {
    targetingOffsetY = Math.abs(targetingOffsetY);
  }
}
