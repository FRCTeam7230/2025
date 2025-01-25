
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SmartMotorController;

public class ElevatorSubsystem extends SubsystemBase {
    private static double kDt = 0.02;

    private final SmartMotorController m_motor = new SmartMotorController(1);
    // Note: These gains are fake, and will have to be tuned for your robot.
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1, 1);
  
    // Create a motion profile with the given maximum voltage and characteristics kV, kA
    // These gains should match your feedforward kV, kA for best results.
    private final ExponentialProfile m_profile =
        new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10, 1, 1));
    private ExponentialProfile.State m_goal = new ExponentialProfile.State(0, 0);
    private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

    public ElevatorSubsystem() {
        // Note: These gains are fake, and will have to be tuned for your robot.
        m_motor.setPID(1.3, 0.0, 0.7);
    }

    public void reachGoal(double goal) {
        var goalState = new ExponentialProfile.State(goal, 0);
    
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves
        // toward the goal while obeying the constraints.
        ExponentialProfile.State next = m_profile.calculate(kDt, m_setpoint, m_goal);

        // Send setpoint to offboard controller PID
        m_motor.setSetpoint(
            SmartMotorController.PIDMode.kPosition,
            m_setpoint.position,
            m_feedforward.calculate(next.velocity) / 12.0);

        m_setpoint = next;
    }
    
    @Override
    public void periodic() {
        //m_motor.updateTelemetry();
    }
}
