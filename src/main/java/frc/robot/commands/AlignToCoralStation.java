// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralStationAlignConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlignToCoralStation extends Command {
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;

  LimelightConstants.reefAlignSide alignSide;

  double xTarget;
  double yTarget;
  double yawTarget;
  PIDController xController;
  PIDController yController;
  PIDController yawController;

  public AlignToCoralStation(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);

    xController = new PIDController(CoralStationAlignConstants.kDriveKp,0,0);
    yController = new PIDController(CoralStationAlignConstants.kDriveKp,0,0);
    yawController = new PIDController(CoralStationAlignConstants.kRotationKp,0,0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    xTarget = CoralStationAlignConstants.kTargetX;
    yTarget = CoralStationAlignConstants.ktargetY;
    yawTarget = CoralStationAlignConstants.kTargetYaw;

    /* TODO: Implement automatic coral station choosing
    Pose2d startPose = m_drive.getPose();
    if(startPose.getX()>CoralStationAlignConstants.fieldCenterX)
    {
      xTarget = 2*CoralStationAlignConstants.fieldCenterX-xTarget;
      yawTarget = 180-yawTarget;
    }
    if(startPose.getY()>CoralStationAlignConstants.fieldCenterY)
    {
      yTarget = 2*CoralStationAlignConstants.fieldCenterY-yTarget;
      yawTarget*=-1;
    }
      */

    xController.setTolerance(CoralStationAlignConstants.kPositionErrorThreshold);
    yController.setTolerance(CoralStationAlignConstants.kPositionErrorThreshold);
    yawController.setTolerance(CoralStationAlignConstants.kRotationErrorThreshold);

    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    yawController.setSetpoint(yawTarget);

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
    Pose2d robotPose = m_drive.getPose();
    //ensures valid targetdata
    if(robotPose!=null)
    {
      m_drive.resetOdometry(m_drive.getPose());
      double xPos = robotPose.getX();
      double yPos = robotPose.getY();
      Rotation2d yaw = robotPose.getRotation();

      
      double xValue = xController.calculate(xPos);
      double yValue = yController.calculate(yPos);
      double yawValue = yawController.calculate(yaw.getDegrees());

      SmartDashboard.putNumber("X Error",xController.getError());
      SmartDashboard.putNumber("Y Error",yController.getError());
      SmartDashboard.putNumber("Yaw Error",yawController.getError());
      
      SmartDashboard.putNumber("X Output",xValue);
      SmartDashboard.putNumber("Y Output",yValue);
      SmartDashboard.putNumber("Yaw Output",yawValue);

      //drive x,z,yaw values
      m_drive.drive(xValue, yValue, yawValue, true, true);
  
      
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
     * if error is below threshold we return true
     */

     if(xController.atSetpoint()&&yawController.atSetpoint()&&yController.atSetpoint())
     {
       return true;
     }
    return false;
  }
  
  
}
