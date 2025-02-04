// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystemYAGSL extends SubsystemBase
{

  // This gearbox represents a gearbox containing 1 Neo
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  private final SparkMax                  m_motor      = new SparkMax(4, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();
  private final SparkMaxSim               m_motorSim   = new SparkMaxSim(m_motor, m_elevatorGearbox);

  // Sensors
  private final LaserCan         m_elevatorLaserCan          = new LaserCan(0);
  private final LaserCanSim      m_elevatorLaserCanSim       = new LaserCanSim(0);
  private final double           m_laserCanOffsetMillimeters = Inches.of(3).in(Millimeters);
  private final RegionOfInterest m_laserCanROI               = new RegionOfInterest(0, 0, 16, 16);
  private final TimingBudget     m_laserCanTimingBudget      = TimingBudget.TIMING_BUDGET_20MS;
  private final Alert            m_laserCanFailure           = new Alert("LaserCAN failed to configure.",
                                                                         AlertType.kError);

  private final DigitalInput m_limitSwitchLow    = new DigitalInput(1);
  private       DIOSim       m_limitSwitchLowSim = null;

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinRealElevatorHeightMeters,
          ElevatorConstants.kMaxRealElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  // Create a Mechanism2d visualization of the elevator
/*
  private final Mechanism2d         m_mech2d         = new Mechanism2d(20, 12);
  private final MechanismRoot2d     m_mech2dRoot     = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));
*/
  private final Mechanism2d m_mech2d =
      new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(51));
  private final MechanismRoot2d m_mech2dRoot =
      m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystemYAGSL()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(convertDistanceToRotations(Meters.of(1)).per(Second).in(RPM))
        .maxAcceleration(convertDistanceToRotations(Meters.of(2)).per(Second).per(Second)
                                 .in(RPM.per(Second)));
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);

    try
    {
      m_elevatorLaserCanSim.setRangingMode(RangingMode.LONG);
    } catch (Exception e)
    {
      m_laserCanFailure.set(true);
    }

    if (RobotBase.isSimulation())
    {
      m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
      SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchLow);
      seedElevatorMotorPosition();
    }
  }

  /**
   * Advance the simulation.
   */
  public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.iterate(
        convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond())).per(Second).in(RPM),
        RoboRioSim.getVInVoltage(),
        0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update lasercan sim.
    m_elevatorLaserCanSim.setMeasurementFullSim(new Measurement(
        LASERCAN_STATUS_VALID_MEASUREMENT,
        (int) (Math.floor(Meters.of(m_elevatorSim.getPositionMeters()).in(Millimeters)) +
               m_laserCanOffsetMillimeters),
        0,
        true,
        m_laserCanTimingBudget.asMilliseconds(),
        m_laserCanROI
    ));

  }

  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedElevatorMotorPosition()
  {
    if (RobotBase.isSimulation())
    {
      // Get values from Simulation
      Measurement measurement = m_elevatorLaserCanSim.getMeasurement();
      // Change distance field
      measurement.distance_mm = (int) (Math.floor(Meters.of(m_elevatorSim.getPositionMeters()).in(Millimeters)) -
                                       m_laserCanOffsetMillimeters);
      // Update simulation distance field.
      m_elevatorLaserCanSim.setMeasurementFullSim(measurement);

      m_encoder.setPosition(convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCanSim.getMeasurement().distance_mm + m_laserCanOffsetMillimeters))
                                    .in(Rotations));
    } else
    {
      m_encoder.setPosition(convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCan.getMeasurement().distance_mm + m_laserCanOffsetMillimeters))
                                    .in(Rotations));
    }
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    m_controller.setReference(convertDistanceToRotations(Meters.of(goal)).in(Rotations),
                              ControlType.kMAXMotionPositionControl,
                              ClosedLoopSlot.kSlot0,
                              m_feedforward.calculate(
                                  convertRotationsToDistance(Rotations.of(m_encoder.getVelocity())).per(Minute)
                                          .in(MetersPerSecond)));
  }


  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    return convertRotationsToDistance(Rotations.of(m_encoder.getPosition())).in(Meters);
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeight(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop(){
    m_motor.set(0);   
  }
  public void normalUp(){
    m_motor.set(1);   
    reachGoal(Constants.ElevatorSimConstants.kMaxElevatorHeightMeters);
  }
  public void normalDown(){
    m_motor.set(1);
    reachGoal(Constants.ElevatorSimConstants.kMinElevatorHeightMeters);
  }
  public void slowUp(){
    m_motor.set(0.25);
    reachGoal(Constants.ElevatorSimConstants.kMaxElevatorHeightMeters);
  }
  public void slowDown(){
    m_motor.set(0.25);
    reachGoal(Constants.ElevatorSimConstants.kMinElevatorHeightMeters);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(RobotBase.isSimulation() ? m_elevatorSim.getPositionMeters() : m_encoder.getPosition());
  }

  @Override
  public void periodic()
  {
    updateTelemetry();
  }

    /**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance)
    {
      return Rotations.of(distance.in(Meters) /
                          (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI) *
                          ElevatorConstants.kElevatorGearing);
    }

    /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations)
    {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI));
    }
}