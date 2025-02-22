// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.geometry.Pose2d;


public class LimelightSubsystem extends SubsystemBase {


  private double[] targetData;
  private boolean targetValid;
  private Pose2d pose;
  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {}
  
  public void RefreshData()
  {
    targetData= LimelightHelpers.getCameraPose_TargetSpace(LimelightConstants.kLimelightName);
    targetValid = LimelightHelpers.getTV(LimelightConstants.kLimelightName);
    pose = LimelightHelpers.getBotPose2d(LimelightConstants.kLimelightName);
  }

  @Override
  public void periodic()
  {
    RefreshData();
    if(targetValid)
    {
      SmartDashboard.putNumberArray("Apriltag data",targetData);
      SmartDashboard.putNumber("tx",targetData[0]);
      SmartDashboard.putNumber("tz",targetData[2]);
      SmartDashboard.putNumber("yaw",targetData[4]);
    }
  }
  public double[] getPose()
  {
    return targetData;
  }
  public boolean isTV()
  {
    return targetValid;
  }
  public Pose2d getPose2d()
  {
    return pose;
  }

  
}
