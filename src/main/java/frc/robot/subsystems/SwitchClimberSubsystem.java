package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwitchClimberSubsystem extends SubsystemBase
{
  private final SparkMax                  m_motor1        = new SparkMax(Constants.ClimberConstants.kClimbMotor1, MotorType.kBrushless);
  private final SparkMax                  m_motor2        = new SparkMax(Constants.ClimberConstants.kClimbMotor2, MotorType.kBrushless);
  private final SparkMax                  m_wheelMotor    = new SparkMax(Constants.ClimberConstants.kClimbMotor3, MotorType.kBrushless);
  private final SparkMaxConfig            m_config_motor1 = new SparkMaxConfig();
  private final SparkMaxConfig            m_config_motor2 = new SparkMaxConfig();
  private final SparkMaxConfig            m_config_motor3 = new SparkMaxConfig();
  DigitalInput m_toplimitSwitch = new DigitalInput(Constants.ClimberConstants.kClimbSwitch1);
  DigitalInput m_bottomlimitSwitch = new DigitalInput(Constants.ClimberConstants.kClimbSwitch2);
  boolean goal;


  // Set up publishers to Advatage Scope
  BooleanPublisher publisher_topSwitch = NetworkTableInstance.getDefault().getBooleanTopic("Climb/publisher_").publish();
  BooleanPublisher publisher_bottomSwitch = NetworkTableInstance.getDefault().getBooleanTopic("Climb/bottomSwitch").publish();
  
  // Constructor
  public SwitchClimberSubsystem()
  {
    //Set up motor configs
    m_config_motor1.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor2.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_config_motor3.idleMode(SparkBaseConfig.IdleMode.kBrake);


    //Configure motors
    m_config_motor1.disableFollowerMode();
    m_motor1.configure(m_config_motor1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_config_motor2.follow(m_motor1,true);
    m_motor2.configure(m_config_motor2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    m_wheelMotor.configure(m_config_motor3, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    goal = false;

  }

  public Command gotopublisher_(){
    return new RunCommand(
      () -> {
        goal = true;
      });
  }

  public Command gotoBottomSwitch(){
    return new RunCommand(
      () -> {
        goal = false;
      });
  }

  public void motorStop(){
    m_motor1.set(0);
  }

  public void climberWheelStop(){
    m_wheelMotor.set(0);
  }

  //These are good to use the set function
  public void ManualClimberUp(){
    m_motor1.set(Constants.ClimberConstants.kClimbMotorSpeed);
  }
  
  public void ManualClimberDown(){
    m_motor1.set(Constants.ClimberConstants.kClimbMotorSpeedDown);
  }

  public void ManualClimberWheel(){
    m_wheelMotor.set(Constants.ClimberConstants.kClimbMotorSpeed);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */

  @Override
  public void periodic() {
    // Add useful info to dashboard(s)
    publisher_topSwitch.set(m_toplimitSwitch.get());
    publisher_bottomSwitch.set(m_bottomlimitSwitch.get());

    if(goal){
      m_motor1.set(Constants.ClimberConstants.kClimbMotorSpeed);
      if(m_toplimitSwitch.get()){
        m_motor1.set(Constants.ClimberConstants.kClimbWheelSpeed);
        m_wheelMotor.set(Constants.ClimberConstants.kClimbMotorSpeed);
      }
    } else{
      m_wheelMotor.set(0);
      m_motor1.set(Constants.ClimberConstants.kClimbMotorSpeedDown);
      if(m_bottomlimitSwitch.get()){
        m_motor1.set(0);
      }
    }
  }  
}
