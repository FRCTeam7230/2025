// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystemYAGSL;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;

import frc.robot.subsystems.ElevatorSubsystemSim;
import frc.robot.subsystems.SwerveSubsystemSim;
import frc.robot.subsystems.UsbCameraSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.util.Units;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final UsbCameraSubsystem m_UsbCamera = new UsbCameraSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_UsbCamera);
  private final SmartDashboardSubsystem m_smartDashboard = new SmartDashboardSubsystem();
  // private final SwerveSubsystemSim m_robotDrive = new SwerveSubsystemSim();

  DriveSubsystem m_robotDrive;

  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  //private final ElevatorSubsystemYAGSL m_elevator = new ElevatorSubsystemYAGSL();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
  // Logitech joystick controller.
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (RobotBase.isReal()) {
      m_robotDrive = new DriveSubsystem();
    }
    else {
      m_robotDrive = new SwerveSubsystemSim();
    }

    m_elevator.setGoal(0);

    // Register named commands

    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands
    
    
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
                true,
                m_driverController.getThrottle()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Constants.OperatorConstants.BRAKE_BUTTON)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 3) 
        .whileTrue(new RunCommand(
          () -> m_robotDrive.drive(0, Constants.slowSpeedMode, 0, false, m_driverController.getThrottle()), m_robotDrive));

    new JoystickButton(m_driverController, 4) 
        .whileTrue(new RunCommand(
          () -> m_robotDrive.drive(0, -Constants.slowSpeedMode, 0, false, m_driverController.getThrottle()), m_robotDrive));
    new JoystickButton(m_driverController, 12)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.spinAngle(30)));

    new JoystickButton(m_driverController, 11)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.spinAngle(0)));    

    new JoystickButton(m_driverController, 5) 
        .whileTrue(new RunCommand(
          () -> m_robotDrive.drive(Constants.slowSpeedMode, 0, 0, false, m_driverController.getThrottle()), m_robotDrive));
          
    new JoystickButton(m_driverController, 6) 
          .whileTrue(new RunCommand(
          () -> m_robotDrive.drive(-Constants.slowSpeedMode, 0, 0, false, m_driverController.getThrottle()), m_robotDrive));

    new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_UP_BUTTON)
        .whileTrue(new RunCommand(
          () -> m_elevator.reachGoal(Constants.ElevatorSimConstants.kMaxElevatorHeightMeters),
          m_elevator));

    new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_DOWN_BUTTON)
    .whileTrue(new RunCommand(
      () -> m_elevator.reachGoal(Constants.ElevatorSimConstants.kMinElevatorHeightMeters),
      m_elevator));


    //m_elevator.atHeight(5, 0.1).whileTrue(Commands.print("Elevator Command!"));



    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    //Add more paths here.
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));
    SmartDashboard.putData("Path to Knock off Algaes", new PathPlannerAuto("Path to Knock off Algaes"));
    SmartDashboard.putData("Coral 1 Cycle", new PathPlannerAuto("Coral 1 Cycle"));
    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = m_robotDrive.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
    
    SmartDashboard.putData("Toggle camera overlay",m_UsbCamera.toggleOverlay());
    SmartDashboard.putData("Toggle camera flip",m_UsbCamera.toggleFlip());

    SmartDashboard.putData("Start Main Camera",m_UsbCamera.StartCameraFeed(0));
    SmartDashboard.putData("Start Alternate Camera Feed",m_UsbCamera.StartCameraFeed(1));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    // Autos auto = new Autos(m_robotDriveSim);
    // return auto.getAutonomousCommand();
/*
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
*/
  }
}
