// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.DriveSubsystem;


public class LocationSubsystem extends SubsystemBase {
  /** Creates a new LocationSubsystem. */
  private Pose2d currentPosition;
  private LimelightSubsystem m_limelight;
  private DriveSubsystem m_drive;
  public LocationSubsystem(LimelightSubsystem limelight,DriveSubsystem drive)
   {
    m_limelight = limelight;
    m_drive = drive;
   }
  public Pose2d getPosition()
  {
    return currentPosition;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_limelight.isTV())
    {
      Pose2d pose = m_limelight.getPose2d();
      if(pose!=null)
      {
        m_drive.resetOdometry(pose);

      }
    }
  }
}
