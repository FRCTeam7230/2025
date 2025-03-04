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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystemSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems
  DriveSubsystem m_robotDrive;
  ElevatorSubsystem m_elevator;
  IntakeSubsystem m_intake;
  private Boolean fieldRelative = true;


  // The driver's controller
  // XboxController m_driverController = new
  // XboxController(OIConstants.kDriverControllerPort);

  // Logitech joystick controller.
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  BooleanPublisher mode_publisher = NetworkTableInstance.getDefault().getBooleanTopic("Is Field Relative").publish();


  private final SendableChooser<Command> autoChooser;

  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private boolean isCompetition = true;//What replaces this?

  public RobotContainer() {
    
    //Set up Subsystems
    if (RobotBase.isReal()) {
      m_robotDrive = new DriveSubsystem();
      m_elevator = new ElevatorSubsystem();
      m_intake = new IntakeSubsystem();
    } else {
      m_robotDrive = new SwerveSubsystemSim();
      m_elevator = new ElevatorSubsystem();
    }


    // Zero/Reset sensors
    m_robotDrive.zeroHeading();
    m_elevator.resetEncoder();

    // Register named commands
    //TODO: Use parallel commands to speed things up if applicible.

    AutoElevatorCommand elevUp = new AutoElevatorCommand(m_elevator,Constants.ElevatorConstants.kL4PreScoringHeightMeters); // TODO: Replace with constants
    AutoElevatorCommand elevDown = new AutoElevatorCommand(m_elevator,Constants.ElevatorConstants.kIntakeElevatorHeightMeters);
    AutoElevatorCommand score = new AutoElevatorCommand(m_elevator,Constants.ElevatorConstants.kL4PostScoringHeightMeters);
    WaitCommand visionAlignAndScoreLeft  = new WaitCommand(1.5); //TODO Replace with set of commands to align, score and drive backwards
    WaitCommand visionAlignAndScoreRight = new WaitCommand(1.5); //TODO Replace with set of commands to align, score and drive backwards

    NamedCommands.registerCommand("Raise Elevator",elevUp);
    NamedCommands.registerCommand("Lower Elevator",elevDown);
    NamedCommands.registerCommand("Score",score);
    NamedCommands.registerCommand("Run Intake", Commands.run(() -> m_intake.runIntakeRollerMotor()));//Is this how it's done?
    NamedCommands.registerCommand("Stop Intake", Commands.run(() -> m_intake.stopIntakeRollerMotor()));//Is this how it's done?
    NamedCommands.registerCommand("Vision Align And Score Left",visionAlignAndScoreLeft);
    NamedCommands.registerCommand("Vision Align And Score Right",visionAlignAndScoreRight);
    // Use event markers as triggers
    // new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));
    // new EventTrigger("Dance").onTrue(Commands.print("This will not be a command where the robot will spin around itself."));
    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("COMP"))
            : stream
        );
    //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure default commands

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_driverController.getY(), 2) * Math.signum(m_driverController.getY()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_driverController.getX(), 2) * Math.signum(m_driverController.getX()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_driverController.getZ(), 2) * Math.signum(m_driverController.getZ()), OIConstants.kDriveDeadband),
                fieldRelative),
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

    new JoystickButton(m_driverController, Constants.OperatorConstants.ZERO_HEADING_BUTTON)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading()));


    new JoystickButton(m_driverController, Constants.OperatorConstants.SLOW_MODE_LEFT)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0, Constants.slowSpeedMode, 0, false),
            m_robotDrive));

    new JoystickButton(m_driverController, Constants.OperatorConstants.SLOW_MODE_RIGHT)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0, -Constants.slowSpeedMode, 0, false),
            m_robotDrive));

    new JoystickButton(m_driverController, Constants.OperatorConstants.SLOW_MODE_FORWARD)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(Constants.slowSpeedMode, 0, 0, false),
            m_robotDrive));

    new JoystickButton(m_driverController, Constants.OperatorConstants.SLOW_MODE_BACKWARD)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(-Constants.slowSpeedMode, 0, 0, false),
            m_robotDrive));

    // new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_SLOW_UP_BUTTON)
    //     .whileTrue(Commands.startEnd(
    //                           () -> m_elevator.ManualElevatorUp(), 
    //                           () -> m_elevator.motorStop(), 
    //                           m_elevator));
    
    // new JoystickButton(m_driverController, Constants.OperatorConstants.WHEEL_CHARACTERIZATION).whileTrue(
    //                     m_robotDrive.wheelRadiusCharacterization());

                              
    new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_SLOW_DOWN_BUTTON)
        .whileTrue(Commands.startEnd(
                              () -> m_elevator.ManualElevatorDown(), 
                              () -> m_elevator.motorStop(), 
                              m_elevator));

    new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_MINHEIGHT)
        .whileTrue(new RunCommand(
            () -> m_elevator.reachGoal(Constants.ElevatorConstants.kIntakeElevatorHeightMeters),
            m_elevator));
                      
    new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_MAXHEIGHT)
        .whileTrue(new RunCommand(
            () -> m_elevator.reachGoal(Constants.ElevatorConstants.kL4PreScoringHeightMeters),
            m_elevator));

    new JoystickButton(m_driverController, Constants.OperatorConstants.ELEVATOR_SCORINGHEIGHT)
        .whileTrue(new RunCommand(
            () -> m_elevator.reachGoal(Constants.ElevatorConstants.kL4PostScoringHeightMeters),
            m_elevator));
    
    new JoystickButton(m_driverController, Constants.OperatorConstants.INTAKE_BUTTON)
        .whileTrue(new StartEndCommand(
        () -> m_intake.runIntakeRollerMotor(), 
        () -> m_intake.stopIntakeRollerMotor(),
        m_intake));

                                  
    // new JoystickButton(m_driverController, Constants.OperatorConstants.SPIN_0)
    //     .whileTrue(Commands.startEnd(
    //         () -> m_elevator.HoverElevator(), 
    //         () -> m_elevator.motorStop(), 
    //         m_elevator));

    // new JoystickButton(m_driverController, Constants.OperatorConstants.SPIN_30)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.spinAngle(30)));
    

    new JoystickButton(m_driverController, Constants.OperatorConstants.ROBOT_RELATIVE)
        .whileTrue(Commands.sequence(
           new InstantCommand(() -> fieldRelative = !fieldRelative, m_robotDrive),
           new InstantCommand(() -> mode_publisher.set(fieldRelative)))
        );

    // m_elevator.atHeight(5, 0.1).whileTrue(Commands.print("Elevator Command!"));

    /*
     * Discrete paths split into parts incase this is needed:
     * new PathPlannerAuto("Bottom Scoring Part 1")
     * new PathPlannerAuto("Bottom Scoring Part 2")
     * new PathPlannerAuto("Start Left Side Part 1")
     * new PathPlannerAuto("Start Left Side Part 2")
     * new PathPlannerAuto("Start Right Side Part 1")
     * new PathPlannerAuto("Start Right Side Part 2")
     * 
     * Note: If we have to add vision command seperately in code, we can do soemthing similar to what I did in fullAuto and include the Vision Align command.
     */
    // Add a button to run the example auto to SmartDashboard, this will also be in
    // the auto chooser built above
    // Add more paths here.
    // SequentialCommandGroup fullAuto = new SequentialCommandGroup();
    // fullAuto.addCommands(new PathPlannerAuto("COMP - Start Center to Left (Processor) Coral Station"));
    // fullAuto.addCommands(new PathPlannerAuto("COMP - Bottom Scoring"));
     SmartDashboard.putData("COMP - Start Center to Left (Processor) Coral Station", new PathPlannerAuto("COMP - Start Center to Left (Processor) Coral Station"));
     SmartDashboard.putData("COMP - Start Center to Right (Our Barge) Coral Station", new PathPlannerAuto("COMP - Start Center to Right (Our Barge) Coral Station"));
     SmartDashboard.putData("COMP - Start Right (Our Barge) Side", new PathPlannerAuto("COMP - Start Right (Our Barge) Side"));
     SmartDashboard.putData("COMP - Start Left (Processor) Side", new PathPlannerAuto("COMP - Start Left (Processor) Side"));
    // // Add a button to run pathfinding commands to SmartDashboard
    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    //     new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
    //     new PathConstraints(
    //         4.0, 4.0,
    //         Units.degreesToRadians(360), Units.degreesToRadians(540)),
    //     0));
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //     new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
    //     new PathConstraints(
    //         4.0, 4.0,
    //         Units.degreesToRadians(360), Units.degreesToRadians(540)),
    //     0));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
        Pose2d currentPose = m_robotDrive.getPose();
  
        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1, -1)), new Rotation2d());
  
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, currentPose.getRotation()));
  
        // Prevent this path from being flipped on the red alliance, since the given
        // positions are already correct
        path.preventFlipping = true;
  
        AutoBuilder.followPath(path).schedule();
      }));

    SmartDashboard.putData("On-the-fly path 2", Commands.runOnce(() -> {
        Pose2d currentPose = m_robotDrive.getPose();
  
        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(-1, 1)), new Rotation2d());
  
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, currentPose.getRotation()));
  
        // Prevent this path from being flipped on the red alliance, since the given
        // positions are already correct
        path.preventFlipping = true;
  
        AutoBuilder.followPath(path).schedule();
      }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_elevator.resetEncoder();//Resets encoder.
    return autoChooser.getSelected();
    // Autos auto = new Autos(m_robotDriveSim);
    // return auto.getAutonomousCommand();
    /*
     * // Create config for trajectory
     * TrajectoryConfig config = new TrajectoryConfig(
     * AutoConstants.kMaxSpeedMetersPerSecond,
     * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
     * // Add kinematics to ensure max speed is actually obeyed
     * .setKinematics(DriveConstants.kDriveKinematics);
     * 
     * // An example trajectory to follow. All units in meters.
     * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
     * // Start at the origin facing the +X direction
     * new Pose2d(0, 0, new Rotation2d(0)),
     * // Pass through these two interior waypoints, making an 's' curve path
     * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
     * // End 3 meters straight ahead of where we started, facing forward
     * new Pose2d(3, 0, new Rotation2d(0)),
     * config);
     * 
     * var thetaController = new ProfiledPIDController(
     * AutoConstants.kPThetaController, 0, 0,
     * AutoConstants.kThetaControllerConstraints);
     * thetaController.enableContinuousInput(-Math.PI, Math.PI);
     * 
     * SwerveControllerCommand swerveControllerCommand = new
     * SwerveControllerCommand(
     * exampleTrajectory,
     * m_robotDrive::getPose, // Functional interface to feed supplier
     * DriveConstants.kDriveKinematics,
     * 
     * // Position controllers
     * new PIDController(AutoConstants.kPXController, 0, 0),
     * new PIDController(AutoConstants.kPYController, 0, 0),
     * thetaController,
     * m_robotDrive::setModuleStates,
     * m_robotDrive);
     * 
     * // Reset odometry to the starting pose of the trajectory.
     * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
     * 
     * // Run path following command, then stop at the end.
     * return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
     * false));
     */
  }
}
