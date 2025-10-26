// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.L1Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignL1 extends Command {
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;

  double forwardOffset;

  PIDController forwardController;
  PIDController yawController;
  LinearFilter filter= LinearFilter.movingAverage(5);

  public AlignL1(DriveSubsystem drive, LimelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_limelight = limelight;
    addRequirements(m_drive,m_limelight);

    forwardController = new PIDController(LimelightConstants.kDriveForwardKp,0,0);
    yawController = new PIDController(LimelightConstants.kRotationKp,0,0);
  }
  @Override
  public void initialize() 
  {
    forwardOffset = L1Constants.kScoreForwardOffset;

    forwardController.setTolerance(LimelightConstants.kPositionErrorThreshold);
    yawController.setTolerance(LimelightConstants.kRotationErrorThreshold);

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
    double[] targetData = m_limelight.getPose();
    //ensures valid targetdata
    if(m_limelight.isTV() && targetData.length>=5)
    {

      double tz = targetData[2];
      double yaw = targetData[4];//Yaw is 0 deg when it's facing tag

      yaw = filter.calculate(yaw);
  
      double zValue = forwardController.calculate(tz);
      double yawValue = yawController.calculate(yaw);

      SmartDashboard.putNumber("L1 z Error",forwardController.getError());
      SmartDashboard.putNumber("L1 Yaw Error",yawController.getError());
      
  
      //drive x,z,yaw values
      m_drive.driveTagRelative(zValue,0,-yawValue,-yaw);
  
      
    }
    else{
      m_drive.drive(0,0 ,0, false, true);
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //disable drive system
    m_drive.drive(0, 0, 0, false, false);
    //maybe trigger scoring

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * get tx, tz, yaw values and calculate error
     * if error is below threshold and at the extended point we return true
     */

     if(yawController.atSetpoint()&&forwardController.atSetpoint())
     {
      return true;
     }
     //or if the target becomes invalid, something went wrong.
     //if(!m_limelight.isTV()) return true; //TODO Remove when it works, replace with teleop driving while not tracking

    return false;
  }
}