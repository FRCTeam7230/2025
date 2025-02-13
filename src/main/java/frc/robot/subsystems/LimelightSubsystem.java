// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightSubsystem extends SubsystemBase {

  private double metersToFeet = 0.3048;
  private double targetingOffsetX;
  private double targetingOffsetZ;

  private double[] targetData;
  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {}
  
  /**
   * @return limelight offset in feet from primary in-view apriltag (asuming apriltag pipeline is active) [forward/backward, left/right, up/down]
   */
  public void RefreshData()
  {
    targetData= LimelightHelpers.getCameraPose_TargetSpace("");
  }
  /**
   * 
   * @param x offset in feet from apriltg of desired position -> negative is further from reef.
   * @param y offset in feet horizontally from apriltag target -> positive is offset to the Right of apriltag. 
   */
  public void setTargetingOffset(double x, double z)
  {
    targetingOffsetX = x;
    targetingOffsetZ = z;
  }
  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("tx",targetData[0]);
  }
  public double[] getPose()
  {
    return targetData;
  }
  public void AlignToTarget()
  {
        RefreshData();
        double tx = targetData[0];
        double tz = targetData[2];
        double yaw = targetData[4];

        tx-=targetingOffsetX;
        tz-=targetingOffsetZ;
  }
  
}
