// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase
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

  private final SparkMax                  m_motor1        = new SparkMax(Constants.ElevatorConstants.kElevMotor1, MotorType.kBrushless);
  private final SparkMax                  m_motor2        = new SparkMax(Constants.ElevatorConstants.kElevMotor2, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller    = m_motor1.getClosedLoopController();
  private final RelativeEncoder           m_encoder       = m_motor1.getEncoder();
  private final SparkMaxSim               m_motorSim      = new SparkMaxSim(m_motor1, m_elevatorGearbox);
  private final SparkClosedLoopController m_controller2   = m_motor2.getClosedLoopController();
  private final RelativeEncoder           m_encoder2      = m_motor2.getEncoder();
  private final SparkMaxSim               m_motorSim2     = new SparkMaxSim(m_motor2, m_elevatorGearbox);
  private final SparkMaxConfig            m_config_motor1 = new SparkMaxConfig();
  private final SparkMaxConfig            m_config_motor2 = new SparkMaxConfig();

  // Simulation classes help us simulate what's going on, including gravity.
  // Question/TODO: Will this be used, or is all sim stuff in the YAGSL class? If no longer relevat, please delete
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.gearRatio,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinRealElevatorHeightMeters,
          ElevatorConstants.kMaxRealElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d         m_mech2d         = new Mechanism2d(20, 12);
  private final MechanismRoot2d     m_mech2dRoot     = m_mech2d.getRoot("Elevator Root", 10, 0);
  /*private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));*/

  /**
   * Subsystem constructor.
   */
  DoublePublisher encoder1_publisher = NetworkTableInstance.getDefault().getDoubleTopic("encoder1value").publish();
  DoublePublisher encoder2_publisher = NetworkTableInstance.getDefault().getDoubleTopic("encoder2value").publish();


  public void motorStop(){
    m_motor1.set(0);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0);
  }

  public ElevatorSubsystem()
  {
  
    m_config_motor1.encoder
        .positionConversionFactor(ElevatorConstants.kRotationToInches); // Converts Rotations to Inches
        //.velocityConversionFactor(0); // Converts RPM to MPS
    m_config_motor1.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)//Change PID with these constants.
        .maxMotion
        .maxVelocity(ElevatorConstants.kElevatorMaxVelocity)
        .maxAcceleration(ElevatorConstants.kElevatorMaxAcceleration)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.1); // TODO: Tune this as we go -see what's reasonable
    m_config_motor1.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor2.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // TODO: set smart current limit. Should be similar to the max current used to sense the top and bottom of the elevator
    // smartCurrentLimit function for both motor configs - set stall only


    m_motor1.configure(m_config_motor1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_config_motor2.follow(m_motor1,true);
    m_motor2.configure(m_config_motor2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    //TODO: Delete if not being used
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /**
   * Advance the simulation.
   */
  /*public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }*/

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    m_controller.setReference(goal,
                              ControlType.kMAXMotionPositionControl,
                              ClosedLoopSlot.kSlot0,
                              m_feedforward.calculate(m_encoder.getVelocity()));

  }


  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    return m_encoder.getPosition();
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

  /*
   * Stop the control loop and motor output.
   */
 

  //These are good to use the set function
  /*public void normalUp(){
    m_motor1.set(-0.25);   
    //reachGoal(Constants.ElevatorSimConstants.kMaxElevatorHeightMeters);
  }
  public void normalDown(){
    m_motor1.set(0.25);
    //reachGoal(Constants.ElevatorSimConstants.kMinElevatorHeightMeters);
  }*/ 
  public void ManualElevatorUp(){
    m_motor1.set(-0.05);
    //reachGoal(Constants.ElevatorSimConstants.kMaxElevatorHeightMeters);
  }

  public void ManualElevatorDown(){
    m_motor1.set(0.05);
    ///reachGoal(Constants.ElevatorSimConstants.kMinElevatorHeightMeters);
  }

  public void smartCurrentLimit() { //i think? this is brake mode - i was wondering if we should put this in. 
    //Answer: Not exactly, the smart current limit is just a way for the motor controller itself to not allow the motors to kill themselves by stalling. 
    // When the configured current is hit, the controllers will stop the motor 
    // (usually wahtever the code was doing to hit that limit restarts the motor, and it sits at that current unti you disable or fic the issue) 
    // You can configure this directly in the constructor with m_config_motor1.smartCurrentLimit(Constants.ElevatorConstants.maxCurrent); and do the same for motor2
    // YOu do not need to check for any limit on your own, just set that parameter and the controller handles the rest
    
    // Brake mode is unrelated - it is the behavior of the motor when the motor is idle (set to zero output) 
    // - brake means that they are harder to move when idled, coast means they're easier to move - we already have this set to brake in the constructor
    if (m_motor1.getOutputCurrent() > Constants.ElevatorConstants.maxCurrent) {
        m_config_motor1.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_config_motor2.idleMode(SparkBaseConfig.IdleMode.kBrake);
    }
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  /*public void updateTelemetry()
  {
    // Update elevator visualization with position
    //m_elevatorMech2d.setLength(m_encoder.getPosition());
  }
*/
  @Override
  public void periodic() {
    encoder1_publisher.set(m_encoder.getPosition());
    //updateTelemetry();
    if (m_motor1.getOutputCurrent() > Constants.ElevatorConstants.resetCurrent) {
      // TODO: Replace get with getAppliedOutput or getBusVoltage - tbd which - done (but will need to test)
      // TODO: Check if up is positive or negative volts
      if (m_motor1.getBusVoltage() < 0) {
        m_encoder.setPosition(Constants.ElevatorConstants.kMaxRealElevatorHeightMeters);
      }
      else {
        m_encoder.setPosition(Constants.ElevatorConstants.kMinRealElevatorHeightMeters);
      }
      m_motor1.set(0);
    }  
    SmartDashboard.putNumber("Elevator Position (Meters?)", m_encoder.getPosition());
  }

}