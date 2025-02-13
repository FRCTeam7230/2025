// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithLimelight extends Command {
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;
  ElevatorSubsystem m_elevator;

  LimelightConstants.reefAlignSide alignSide;

  double horizontalOffset;
  double forwardOffset;

  /** Creates a new AlignWithLimelight. */
  public AlignWithLimelight(DriveSubsystem drive, LimelightSubsystem limelight, ElevatorSubsystem elevator,LimelightConstants.reefAlignSide side) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);

    m_limelight = limelight;
    addRequirements(m_limelight);

    m_elevator = elevator;
    addRequirements(m_elevator);

    alignSide = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(alignSide == LimelightConstants.reefAlignSide.Left)
    {
      horizontalOffset = -1*LimelightConstants.kHorizontalOffset;
    }
    else
    {
      horizontalOffset = LimelightConstants.kHorizontalOffset;
    }
    forwardOffset = LimelightConstants.kForwardUnextendedOffset;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*
 * get tx, tz, yaw values from the limelight subsystem
 * if the elevator is fully extended, our tx is aligned, and the yaw is aligned, switch tot he extended offset
 *  
 * 
 * use pid class to calculate values
 * pass into drive system
 */
    double[] targetData = m_limelight.getPose();
    double tx = targetData[0];
    double tz = targetData[2];
    double yaw = targetData[4];
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //disable drive system
    //maybe trigger scoring

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * get tx, tz, yaw values and calculate error
     * if error is below threshold and at the extended point we return true
     */
    return false;
  }
}
