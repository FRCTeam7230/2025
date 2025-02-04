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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

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

  //TODO: CAN IDs should be constants
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
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightInches,
          ElevatorConstants.kMaxElevatorHeightInches,
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
  DoubleArrayPublisher encoder1_publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("encoder1value").publish();
  DoubleArrayPublisher encoder2_publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("encoder2value").publish();

  public double EncoderCounttoInches(double numRevolution) {
    numRevolution = m_encoder.getPosition()/42;
    double numRevShaft = numRevolution/15;
    double elevHeight = numRevShaft*2*Math.PI*Constants.ElevatorConstants.gearRadius;
    return elevHeight;
  }

  public double InchestoEncoderCount(double height) {
    double revShaft = height/(2*Math.PI*Constants.ElevatorConstants.gearRadius);
    double rev = revShaft*15;
    double encoderCount = rev*42;
    return encoderCount;
  }

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
        .allowedClosedLoopError(0.1); // TODO: 0.01 inches is a very small closed loop error, maybe 0.1? or 0.5?
    m_config_motor1.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor2.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // TODO: set smart current limit. Should be similar to the max current used to sense the top and bottom of the elevator
    // smartCurrentLimit function for both motor configs - set stall only


    m_motor1.configure(m_config_motor1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    // TODO: invert should be true not false, right?
    m_config_motor2.follow(m_motor1,true);
    m_motor2.configure(m_config_motor2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
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
    //TODO: This should be using ControlType kMAXMotionPositionControl
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
  //TODO: Should not stop the closed loop control with these. Rather, should set the goal position to be the 
  // set elevator heights (can use reachGoal function)
  // TODO: THese should be referenceing the real robot heights, not the sim robot heights - fixed
  // Or include logic for if real use the real heights, if sim use the sim heights
  public void OneThird(){
    m_motor1.set(-0.25);
    setGoal(InchestoEncoderCount(Constants.ElevatorConstants.kMaxElevatorHeightInches/3)); //set goal or reach goal?
    m_motor1.stopMotor();
    
    if (m_encoder.getPosition()==InchestoEncoderCount(Constants.ElevatorConstants.kMaxElevatorHeightInches)/3){
      m_motor1.stopMotor();
    }
  }
  public void TwoThird(){
    m_motor1.set(0.25);
    reachGoal(InchestoEncoderCount(Constants.ElevatorConstants.kMaxElevatorHeightInches * 2/3));
    m_motor1.stopMotor();

    if (m_encoder.getPosition()==InchestoEncoderCount(Constants.ElevatorConstants.kMaxElevatorHeightInches) * 2/3){
      m_motor1.stopMotor();
    }
  }

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

  //TODO: Remove this function? It's not used and I'm not sure what exactly it's doing
  public void ToggleMove(){ //need to ask yoshito about this
    if (m_encoder.getPosition()==InchestoEncoderCount(Constants.ElevatorConstants.kMaxElevatorHeightInches)){
      m_motor1.set(0.05);
    } else if (m_encoder.getPosition()==Constants.ElevatorConstants.kMinElevatorHeightInches){
      m_motor1.set(-0.05);
    } else {
      m_motor1.set(0.05);//Go down by default
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
    double [] encoder1value = {(double) m_encoder.getPosition()};
    encoder1_publisher.set(encoder1value);
    //updateTelemetry();
    // TODO: replace getAppliedOutput with getOutputCurrent
    // TODO: Max Current should be set much higher - according to rev the empirical stall current is 105, I think something like 80 would be good
    if (m_motor1.getOutputCurrent()>Constants.ElevatorConstants.currentMax){//Or m_motor.getBusVoltage()
      // TODO: Replace get with getAppliedOutput or getBusVoltage - tbd which
      m_motor1.set(0);
      if (m_motor1.getBusVoltage() < 0) {
        m_encoder.setPosition(Constants.ElevatorConstants.kMaxElevatorHeightInches);
      }

      else {
        m_encoder.setPosition(Constants.ElevatorConstants.kMinElevatorHeightInches);
      }
    }  
    SmartDashboard.putNumber("Elevator Position (Inches)", m_encoder.getPosition());
  }

}