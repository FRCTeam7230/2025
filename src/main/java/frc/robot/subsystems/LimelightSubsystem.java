// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {


  private double[] targetData;
  private boolean targetValid;
  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {}
  
  /**
   * @return limelight offset in feet from primary in-view apriltag (asuming apriltag pipeline is active) [forward/backward, left/right, up/down]
   */
  public void RefreshData()
  {
    targetData= LimelightHelpers.getCameraPose_TargetSpace("");
    targetValid = LimelightHelpers.getTV("");
  }

  @Override
  public void periodic()
  {
    RefreshData();
    if(targetValid)
    {
      SmartDashboard.putNumberArray("Apriltag data",targetData);

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

  
}
