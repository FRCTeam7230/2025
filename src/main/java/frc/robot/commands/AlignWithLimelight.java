// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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

  PIDController xController;
  PIDController forwardController;
  PIDController yawController;
  private boolean targetingExtendedPosition = false;

  /** Creates a new AlignWithLimelight. */
  public AlignWithLimelight(DriveSubsystem drive, LimelightSubsystem limelight, ElevatorSubsystem elevator,LimelightConstants.reefAlignSide side) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_limelight = limelight;
    m_elevator = elevator;
    addRequirements(m_drive,m_limelight,m_elevator);

    alignSide = side;

    xController = new PIDController(LimelightConstants.kDriveKp,0,0);
    forwardController = new PIDController(LimelightConstants.kDriveKp,0,0);
    yawController = new PIDController(LimelightConstants.kRotationKp,0,0);
    targetingExtendedPosition = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    targetingExtendedPosition = false;
    if(alignSide == LimelightConstants.reefAlignSide.Left)
    {
      horizontalOffset = -1*LimelightConstants.kHorizontalOffset;
    }
    else
    {
      horizontalOffset = LimelightConstants.kHorizontalOffset;
    }
    forwardOffset = LimelightConstants.kForwardUnextendedOffset;



    xController.setTolerance(LimelightConstants.kPositionErrorThreshold);
    forwardController.setTolerance(LimelightConstants.kPositionErrorThreshold);
    yawController.setTolerance(LimelightConstants.kRotationErrorThreshold);

    xController.setSetpoint(horizontalOffset);
    forwardController.setSetpoint(forwardOffset);
    yawController.setSetpoint(0);

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
//if elevator is extended, we can go right up to the reef
    if(m_elevator.isFullyExtended(LimelightConstants.kElevatorTolerance)&&xController.atSetpoint()&&yawController.atSetpoint())
    {
      forwardController.setSetpoint(LimelightConstants.kForwardExtendedOffset);
      targetingExtendedPosition = true;
    }
    double[] targetData = m_limelight.getPose();
    //ensures valid targetdata
    if(m_limelight.isTV())
    {

      double tx = targetData[0];
      double tz = targetData[2];
      double yaw = targetData[4];
  
      double xValue = xController.calculate(tx);
      double zValue = forwardController.calculate(tz);
      double yawValue = yawController.calculate(yaw);
      
  
      //drive x,z,yaw values
      m_drive.driveTagRelative(-zValue,-xValue,yawValue,-yaw);
  
      
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //disable drive system
    m_drive.drive(0, 0, 0, false);
    //maybe trigger scoring

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * get tx, tz, yaw values and calculate error
     * if error is below threshold and at the extended point we return true
     */

     if(xController.atSetpoint()&&yawController.atSetpoint()&&forwardController.atSetpoint())
     {
        return targetingExtendedPosition;
     }
     //or if the target becomes invalid, something went wrong.
     if(!m_limelight.isTV()) return true; //TODO Remove when it works, replace with teleop driving while not tracking

    return false;
  }
  
  
}
