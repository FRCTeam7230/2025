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
  public static final double movementDivider = 4;
  public static final double rotateDivider = 5;
  
  public static class L1Constants {
    public static final int kL1Motor = 4;//Choose a port for motor. 

    // L1 TODO - Will need an intake position, a score position, a stow position, and an L4 score position
    public static final double intakePosition  = 21;
    public static final double scorePosition   = 336; // deg
    public static final double stowPosition    = 30; // deg
    public static final double l4ScorePosition = 247; // deg     was 239 change to ~247
    public static final double l4Intake        = stowPosition;

    public static final double kElevMaxHeightForL4Scoring = .5;

    public static final double kScoreForwardOffset = -0.48;

    //public static final double kResetCurrent = 0; //max current tbd TODO: Will need to tune these currents
    public static final int kMaxCurrent   = 20;//In amps? TODO: Will need to tune these currents

    public static final double kL1Kp = 0.012; //TODO: Will need to tune this, I lowered it to start 
    public static final double kL1Ki = 0;
    public static final double kL1Kd = 0; //0.00075;
    public static final double kL1RampRate = 0.2;//not sure what this one is for

    //Note, all should be ok at zero except kG 
    public static final double kL1kS = 0;
    public static final double kL1kV = 0;
    public static final double kL1kG = 0.6;
    public static final double kL1kA = 0;

    public static final double elevatorHeight = 0.5; //This is the constant to determine whether to extend or retract based on elevator height.
    public static final double overrideHeight = 0.6;
    public static final double overrideHeightDown = 0.7;

//     Gear ratio
// Output pulley radius/diameter
// Encoder ticks per revolution
// CAN IDs
// Will need to decide on a motor current limit config value

  }
  public static class ControllerConstants {
    //Centeralized controller constants that the code reads from. If usingXBoxController is true, then the buttons are using the XBoxConstants' button numbers
    public static final boolean usingXBoxController = true;

    // Drive Stick Buttons. 
    public static final int BRAKE_BUTTON = usingXBoxController ? XBoxConstants.BRAKE_BUTTON : JoystickConstants.BRAKE_BUTTON;
    public static final int INTAKE_BUTTON = usingXBoxController ? XBoxConstants.INTAKE_BUTTON : JoystickConstants.INTAKE_BUTTON;
    public static final int SLOW_MODE_LEFT = usingXBoxController ? XBoxConstants.SLOW_MODE_LEFT : JoystickConstants.SLOW_MODE_LEFT;
    public static final int SLOW_MODE_RIGHT = usingXBoxController ? XBoxConstants.SLOW_MODE_RIGHT : JoystickConstants.SLOW_MODE_RIGHT;
    public static final int SCORE_LEFT = usingXBoxController ? XBoxConstants.SCORE_LEFT : JoystickConstants.SCORE_LEFT;
    public static final int SCORE_AUTO = usingXBoxController ? XBoxConstants.SCORE_AUTO : JoystickConstants.SCORE_AUTO;
    public static final int MANUAL_ELEVATOR_DOWN = usingXBoxController ? XBoxConstants.MANUAL_ELEVATOR_DOWN : JoystickConstants.MANUAL_ELEVATOR_DOWN;
    public static final int ZERO_HEADING_BUTTON = usingXBoxController ? XBoxConstants.ZERO_HEADING_BUTTON : JoystickConstants.ZERO_HEADING_BUTTON;
    public static final int MANUAL_L1_DOWN = usingXBoxController ? XBoxConstants.MANUAL_L1_DOWN : JoystickConstants.MANUAL_L1_DOWN;
    public static final int MANUAL_L1_UP = usingXBoxController ? XBoxConstants.MANUAL_L1_UP : JoystickConstants.MANUAL_L1_UP;
    public static final int ELEVATOR_INCREMENT_DOWN = usingXBoxController ? XBoxConstants.ELEVATOR_INCREMENT_DOWN : JoystickConstants.ELEVATOR_INCREMENT_DOWN;
    public static final int ELEVATOR_MAXHEIGHT = usingXBoxController ? XBoxConstants.ELEVATOR_MAXHEIGHT : JoystickConstants.ELEVATOR_MAXHEIGHT;
    public static final int ROBOT_RELATIVE = usingXBoxController ? XBoxConstants.ROBOT_RELATIVE : JoystickConstants.ROBOT_RELATIVE;
    public static final int HOVER_L1 = XBoxConstants.HOVER_L1;
    public static final int throttleButton1 = usingXBoxController ? 100 : JoystickConstants.throttleButton1;
    public static final int throttleButton2 = usingXBoxController ? 100 : JoystickConstants.throttleButton2;

  //test joystick buttons for joystick. Used during testing, not updated nor used anymore.
  /*
    public static final int NOT_USED_1 = kButton1;
    public static final int NOT_USED_2 = kButton2;
    public static final int NOT_USED_3 = kButton3;
    public static final int NOT_USED_4 = kButton4;  
    public static final int NOT_USED_5 = kButton5;
    public static final int NOT_USED_6 = kButton6;
    public static final int SPIN_30= kButton7;
    public static final int MANUAL_UP = kButton8;
    public static final int SPIN_0 = kButton9;
    public static final int HOVER_ELEVATOR = kButton10;
    public static final int WHEEL_CHARACTERIZATION = kButton11;
    public static final int MANUAL_DOWN = kButton12;
    */

  }

  public static class JoystickConstants {
    //Button mappings for the joystick
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

    //These are the numbers that are used for the ButtonMappings check, these must be unique numbers. Ahana chose these, not me
    public static final int throttleButton1 = 15; // for when below -0.7
    public static final int throttleButton2 = 30; // for above 0.7

    // Joystick driving mappings
    public static final int BRAKE_BUTTON = kButton1;
    public static final int INTAKE_BUTTON = kButton2;
    public static final int SLOW_MODE_LEFT = kButton3;
    public static final int SLOW_MODE_RIGHT = kButton4;
    public static final int SCORE_LEFT = kButton5;
    public static final int SCORE_AUTO = kButton6;
    public static final int MANUAL_ELEVATOR_DOWN = kButton7;
    public static final int ZERO_HEADING_BUTTON = kButton8;
    public static final int MANUAL_L1_UP = kButton9;
    public static final int MANUAL_L1_DOWN = kButton10;
    public static final int ELEVATOR_INCREMENT_DOWN = kButton11;
    public static final int ELEVATOR_MAXHEIGHT = kButton12;
    public static final int ROBOT_RELATIVE = 100; // using the throttle buttons for now
  }

  public static class XBoxConstants{
    //Button configurations for the XBox controller
    public static final int kButton1 = 1; //A
    public static final int kButton2 = 2; //B
    public static final int kButton3 = 3; //X
    public static final int kButton4 = 4; //Y
    public static final int kButton5 = 5; //LB, left button
    public static final int kButton6 = 6; //RB, right button
    public static final int kButton7 = 7; //Screenshare button, probably dont use   //not used
    public static final int kButton8 = 8; //Menu button, probably dont use, also the back button
    public static final int kButton9 = 9; //Pressing down left joystick DO NOT USE
    public static final int kButton10 = 10; //Pressing down right joystick DO NOT USE  //not used

    /**
     * Setting the numbers of the povs to literally anything else will probably break everything 
     */
    public static final int pov0 = 0; //up
    public static final int pov45 = 45; //up right
    public static final int pov90 = 90; //right          probably make it elevator intake
    public static final int pov135 = 135; //down right
    public static final int pov180 = 180; //down
    public static final int pov225 = 225; //down left
    public static final int pov270 = 270; //left
    public static final int pov315 = 315; //up left

    public static final int leftTrigger = -2; //LT, left trigger 
    public static final int rightTrigger = -3; //RT, right trigger

    public static final int leftStick_XAXIS = 0;
    public static final int leftStick_YAXIS = 1;
    public static final int rightStick_XAXIS = 4;
    public static final int rightStick_YAXIS = 5;

    // Xbox controller mappings
    public static final int BRAKE_BUTTON = kButton3;
    public static final int INTAKE_BUTTON = kButton9;

    public static final int SCORE_AUTO = 67; //score right now
    public static final int MANUAL_ELEVATOR_DOWN = pov270;
    
    public static final int ZERO_HEADING_BUTTON = kButton7; 

    public static final int L1_SCORE = kButton6;
    
    public static final int ELEVATOR_INCREMENT_DOWN = kButton1;
    public static final int ELEVATOR_MAXHEIGHT = pov0;
    public static final int L4_INTAKE = kButton4; //connect
    public static final int L1_INTAKE = kButton5; //connect

    //graveyard
    public static final int MANUAL_L1_DOWN = 67;   //dead
    public static final int MANUAL_L1_UP = 67;     //dead
    public static final int L1_STOW = 67;          //dead
    public static final int SLOW_MODE_LEFT = 67;   //dead
    public static final int SLOW_MODE_RIGHT = 67;  //dead
    public static final int SCORE_LEFT = leftTrigger;       //dead
    public static final int SCORE_RIGHT = rightTrigger; //changed to exist


    public static final int ROBOT_RELATIVE = kButton2;
    public static final int HOVER_L1 = pov90;
    //button 7 is not used
    
    // XBox movement mappings
    public static final int MOVE_XAXIS = leftStick_XAXIS;
    public static final int MOVE_YAXIS = leftStick_YAXIS;
    public static final int MOVE_ZAXIS = rightStick_XAXIS;
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
    public static final int kTestControllerPort = 1;
  
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
    public static final double kElevatorRampRate = 0.2; // Time to get from 0 to full power

    //elevator score slow
    public static final double kSlowElevatorKp = 4.5; //TODO: Will need to tune this, I lowered it to start 
    public static final double kSlowElevatorKi = 0;
    public static final double kSlowElevatorKd = 1;
    public static final double kSlowElevatorRampRate = 0.2;

    // Note: All of these should be 0.0 except kG - which we will need to determine empirically
    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.35; // volts (V) //TODO: Will need to tune
    public static final double kElevatorkV = 0.0; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kCarriageMass = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinRealElevatorHeightMeters = 0;    // m
    public static final double kMaxRealElevatorHeightMeters = 1.575; // m
    
    public static final double kL4PreScoringHeightMeters = 1.558; 
    public static final double slowModeThreshHold = 0.67; 
    //public static final double kL4PostScoringHeightNoAlgeaMeters = kL4PreScoringHeightMeters-0.3; //Are we using this constant?
    public static final double kL4PostScoringHeightMeters = kL4PreScoringHeightMeters-0.3;//0.43
    public static final double kL1ScoringHeightMeters = 0.61;// 66-67

    public static final double kIntakeElevatorHeightMeters = 0.04;    //Whatever the intake height is for L4
    public static final double kL1IntakeElevatorHeightMeters = 0.74; // TODO: Set the value. Whatever the intake height is for L1

    public static final double kRotationToMeters = kGearCircumference / kGearRatio;// Revolutions to Output units conversion factor
    public static final double kElevatorMaxVelocity = 4000; // Motor RPM - does not get affected by conversion factor TODO: Need a good inches per sec max, start slow (10?)
    public static final double kElevatorMaxAcceleration = 4000; // Mo tor RPM - does not get affected by conversion factor TODO: Need a good inches per sec per sec max, start slow (10?)
  
  }

  
  public static class IntakeConstants{
    public static final int intakeRollerMotorID = 2;
  }
  public static class LimelightConstants
  {
    public static final String kLimelightName = "limelight";

    public static final double kDriveForwardKp = 2.4;
    public static final double kDriveHorizontalKp = 1.6; //output = -1 to 1, .1 m off want .1 m/s, 0.1 m/s = 0.04 % output, .04 = kp*0.1, kp = .4
    public static final double kRotationKp = 0.06; //output = -1 to 1, 15 deg off want 60 deg/sec, 60 deg/sec = 1.0 % output, 1.0 = kp*15deg, kp = 0.06

    public static final double kHorizontalOffset = 0.18; // 0.19m
    public static final double kForwardExtendedOffset = -0.47; //TODO: -0.47 at comp! recorded 0.52m
    public static final double kForwardUnextendedOffset = -0.7;

    public static final double kPositionErrorThreshold = 0.05;
    public static final double kRotationErrorThreshold = 1; // deg
    public static final double kElevatorTolerance = 0.02; //m

    public enum reefAlignSide
    {
      Right,
      Left,
    }
  }
}
