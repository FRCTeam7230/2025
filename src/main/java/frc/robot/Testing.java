/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/MAXMotion/src/main/java/frc/robot/Robot.java

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Testing extends TimedRobot {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private SparkMax motor2;
  private SparkMaxConfig motorConfig2;
  private SparkClosedLoopController closedLoopController2;
  private RelativeEncoder encoder2;
  public Testing() {
    /*
     * Constants cuz why not
     */
      double[] Pid0 = {0.4,0,0};
      double[] outputRange0 = {-1,1};

      double[] Pid1 = {0.0001,0,0};
      double velocityFF1 = 1.0 / 5767;
      double[] outputrange1 = {-1.1};

      double maxVelocity0 = 1000;
      double maxAcceleration0 = 1000;
      int allowedClosedLoopError0 = 1;

      double maxVelocity1 = 500;
      double maxAcceleration1 = 6000;
      int allowedClosedLoopError1 = 1;
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */

    motor = new SparkMax(1, MotorType.kBrushless);
    motor2 = new SparkMax(1, MotorType.kBrushless);

    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    closedLoopController2 = motor2.getClosedLoopController();
    encoder2 = motor2.getEncoder();
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();
    motorConfig2 = new SparkMaxConfig();
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    motorConfig2.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(Pid0[0])
        .i(Pid0[1])
        .d(Pid0[2])
        .outputRange(outputRange0[0], outputRange0[1])
        // Set PID values for velocity control in slot 1
        .p(Pid1[0], ClosedLoopSlot.kSlot1)
        .i(Pid1[1], ClosedLoopSlot.kSlot1)
        .d(Pid1[2], ClosedLoopSlot.kSlot1)
        .velocityFF(velocityFF1, ClosedLoopSlot.kSlot1)
        .outputRange(outputrange1[0], outputrange1[1], ClosedLoopSlot.kSlot1);
    
    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(maxVelocity0)
        .maxAcceleration(maxAcceleration0)
        .allowedClosedLoopError(allowedClosedLoopError0)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(maxAcceleration1, ClosedLoopSlot.kSlot1)
        .maxVelocity(maxVelocity1, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(allowedClosedLoopError1, ClosedLoopSlot.kSlot1);
    
    
    
    motorConfig2.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(Pid0[0])
        .i(Pid0[1])
        .d(Pid0[2])
        .outputRange(outputRange0[0], outputRange0[1])
        // Set PID values for velocity control in slot 1
        .p(Pid1[0], ClosedLoopSlot.kSlot1)
        .i(Pid1[1], ClosedLoopSlot.kSlot1)
        .d(Pid1[2], ClosedLoopSlot.kSlot1)
        .velocityFF(velocityFF1, ClosedLoopSlot.kSlot1)
        .outputRange(outputrange1[0], outputrange1[1], ClosedLoopSlot.kSlot1);
    
    motorConfig2.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(maxVelocity0)
        .maxAcceleration(maxAcceleration0)
        .allowedClosedLoopError(allowedClosedLoopError0)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(maxAcceleration1, ClosedLoopSlot.kSlot1)
        .maxVelocity(maxVelocity1, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(allowedClosedLoopError1, ClosedLoopSlot.kSlot1);
    
    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motor2.configure(motorConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  @Override
  public void teleopPeriodic() {
    if (SmartDashboard.getBoolean("Control Mode", false)) {
      /*
       * Get the target velocity from SmartDashboard and set it as the setpoint
       * for the closed loop controller with MAXMotionVelocityControl as the
       * control type.
       */
      double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
      closedLoopController.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot1);
      closedLoopController2.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot1);
    } else {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller with MAXMotionPositionControl as the
       * control type.
       */
      double targetPosition = SmartDashboard.getNumber("Target Position", 0);
      closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
      closedLoopController2.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    }
  }

  @Override
  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putNumber("Actual Position", encoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Actual Position", encoder2.getPosition());
    SmartDashboard.putNumber("Actual Velocity", encoder2.getVelocity());
    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      encoder.setPosition(0);
      encoder2.setPosition(0);
    }
  }
}