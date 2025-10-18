// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ClimberSubsystem extends SubsystemBase
{
  // Set up elevator properties
  private final SparkMax                  m_motor1        = new SparkMax(Constants.ClimberConstants.kClimbMotor1, MotorType.kBrushless);
  private final SparkMax                  m_motor2        = new SparkMax(Constants.ClimberConstants.kClimbMotor2, MotorType.kBrushless);
  private final SparkMax                  m_wheelMotor    = new SparkMax(Constants.ClimberConstants.kClimbMotor3, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller    = m_motor1.getClosedLoopController();
  private final AbsoluteEncoder           m_encoder       = m_motor1.getAbsoluteEncoder();
  private final SparkMaxConfig            m_config_motor1 = new SparkMaxConfig();
  private final SparkMaxConfig            m_config_motor2 = new SparkMaxConfig();
  private final SparkMaxConfig            m_config_motor3 = new SparkMaxConfig();
  double m_desiredHeight;


  // Set up publishers to Advatage Scope
  DoublePublisher encoder1_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/encoder1value").publish();
  DoublePublisher encoder2_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/encoder2value").publish();
  DoublePublisher velocity_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/velocity").publish();
  DoublePublisher output1_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/outputMotor1").publish();
  DoublePublisher output2_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/outputMotor2").publish();
  DoublePublisher heightError_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/heightError").publish();
  DoublePublisher current1_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/currentMotor1").publish();
  DoublePublisher current2_publisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber/currentMotor2").publish();
  BooleanPublisher elevReset_publisher = NetworkTableInstance.getDefault().getBooleanTopic("Climber/resetElev").publish();
  
  // Constructor
  public ClimberSubsystem()
  {
    //Set up motor configs
    m_config_motor1.absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    m_config_motor1.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, ClosedLoopSlot.kSlot0)//Change PID with these constants.
        .pid(ElevatorConstants.kSlowElevatorKp, ElevatorConstants.kSlowElevatorKi, ElevatorConstants.kSlowElevatorKd, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0) //TODO: RESET TO -1 to 1!!!!!
        .outputRange(-0.4, 0.4, ClosedLoopSlot.kSlot1);
    m_config_motor1.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor2.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor3.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor1.smartCurrentLimit(Constants.ClimberConstants.kMaxCurrent);
    m_config_motor2.smartCurrentLimit(Constants.ClimberConstants.kMaxCurrent);
    m_config_motor1.closedLoopRampRate(Constants.ClimberConstants.kClimbRampRate);
    m_config_motor2.closedLoopRampRate(Constants.ClimberConstants.kClimbRampRate);


    //Configure motors
    m_config_motor1.disableFollowerMode();
    m_motor1.configure(m_config_motor1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_config_motor2.follow(m_motor1,true);
    m_motor2.configure(m_config_motor2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // desired height
    m_desiredHeight = 0;

  }

  public void motorStop(){
    m_motor1.set(0);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(boolean goal)
  {
    m_desiredHeight = goal ? Constants.ClimberConstants.kClimbMaxPosition : Constants.ClimberConstants.kClimbMinPosition;
    if (!goal) {
      m_controller
      .setReference(Constants.ClimberConstants.kClimbMinPosition,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot1
        );
    }
    else {
      m_controller
      .setReference(Constants.ClimberConstants.kClimbMaxPosition,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0
        );
    }

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
   * @param tolerance Tolerance in Meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(
      () -> MathUtil.isNear(height, getHeight(), tolerance)
      );
  }
  public boolean isFullyExtended(double tolerance)
  {
    return MathUtil.isNear(Constants.ClimberConstants.kClimbMaxPosition, getHeight(), tolerance);
  }
  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(boolean goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor1.set(0.0);
  }

  //These are good to use the set function
  public void ManualClimberUp(){
    m_motor1.set(Constants.ClimberConstants.kClimbMotorSpeed);
  }
  
  public void ManualClimberDown(){
    m_motor1.set(Constants.ClimberConstants.kClimbMotorSpeedDown);
  }

  @Override
  public void periodic() {

    // Add useful info to dashboard(s)
    encoder1_publisher.set(m_encoder.getPosition());
    output1_publisher.set(m_motor1.getAppliedOutput());
    output2_publisher.set(m_motor2.getAppliedOutput());
    heightError_publisher.set(m_desiredHeight - getHeight());
    velocity_publisher.set(m_encoder.getVelocity());
    current1_publisher.set(m_motor1.getOutputCurrent());
    current2_publisher.set(m_motor2.getOutputCurrent());

    SmartDashboard.putNumber("Climber Position (Meters)", m_encoder.getPosition());
    elevReset_publisher.set(false);

    if(isFullyExtended(0.2)){
      m_wheelMotor.set(Constants.ClimberConstants.kClimbWheelSpeed);
    } else {
      m_wheelMotor.set(0);
    }
  }  

}