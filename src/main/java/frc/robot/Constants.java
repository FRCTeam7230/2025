// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double slowSpeedMode = 0.1;
  public static final double movementDivider = 2;
  public static final double rotateDivider = 5;
  
  public static class OperatorConstants {

    public static final int kButton1 = 1;
    public static final int kButton2 = 2;
    public static final int kButton3 = 3;
    public static final int kButton4 = 4;
    public static final int kButton5 = 5;
    public static final int kButton6 = 6;
    public static final int kButton7 = 7;
    public static final int kButton8 = 8;
    public static final int kButton9 = 9;
    public static final int kButton10 = 10;
    public static final int kButton11 = 11;
    public static final int kButton12 = 12;

    // Drive Stick Buttons
    //testing buttons: 2, 7-12
    public static final int BRAKE_BUTTON = kButton1;
    public static final int ZERO_HEADING_BUTTON = kButton2;
    public static final int SLOW_MODE_LEFT = kButton3;
    public static final int SLOW_MODE_RIGHT = kButton4;  
    public static final int SLOW_MODE_FORWARD = kButton5;
    public static final int SLOW_MODE_BACKWARD = kButton6;          
    public static final int WHEEL_CHARACTERIZATION = kButton7;
    public static final int ELEVATOR_MAXHEIGHT = kButton8;
    public static final int ROBOT_RELATIVE = kButton9;   
    public static final int ELEVATOR_SCORINGHEIGHT = kButton10;
    public static final int ELEVATOR_SLOW_DOWN_BUTTON = kButton11;    
    public static final int ELEVATOR_MINHEIGHT = kButton12; 
    
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // Original value: public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontLeftChassisAngularOffset = (Math.PI/2) + Math.PI;

    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;



    // SPARK MAX CAN IDs
/* Original value: 
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;
*/
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 14;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.079;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  
    // Original value: public static final double kDriveDeadband = 0.05;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    // Original value: public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxSpeedMetersPerSecond = 4.8;
  
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class ElevatorConstants
  {
    public static final int kElevMotor1 = 3;
    public static final int kElevMotor2 = 1;

    public static final double kGearRatio = 15 / 2; // Divided by 2 stages 
    public static final double kGearCircumference = Units.inchesToMeters(5.5); //inches - lemme double check with 
    public static final double kResetCurrent = 45; //max current tbd TODO: Will need to tune these currents
    public static final int kMaxCurrent   = 65;//In amps? TODO: Will need to tune these currents

    public static final double kElevatorKp = 4.5; //TODO: Will need to tune this, I lowered it to start 
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 1;
    public static final double kElevatorRampRate = 0.2;

    // Note: All of these should be 0.0 except kG - which we will need to determine empirically
    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.35; // volts (V) //TODO: Will need to tune
    public static final double kElevatorkV = 0.0; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kCarriageMass = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinRealElevatorHeightMeters = 0;    // m
    public static final double kMaxRealElevatorHeightMeters = 1.575; // m

    public static final double kRotationToMeters = kGearCircumference / kGearRatio;// Revolutions to Output units conversion factor
    public static final double kElevatorMaxVelocity = 4000; // Motor RPM - does not get affected by conversion factor TODO: Need a good inches per sec max, start slow (10?)
    public static final double kElevatorMaxAcceleration = 4000; // Motor RPM - does not get affected by conversion factor TODO: Need a good inches per sec per sec max, start slow (10?)
  }
}
