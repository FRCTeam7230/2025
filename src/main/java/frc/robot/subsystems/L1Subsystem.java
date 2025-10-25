package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.L1Constants;

public class L1Subsystem extends SubsystemBase {
    private final SparkMax                  m_motor;//        = new SparkMax(Constants.L1Constants.kL1Motor, MotorType.kBrushed);
    private final AbsoluteEncoder           m_encoder;

    private final SparkMaxConfig            m_motorConfig        = new SparkMaxConfig();
    private final SparkClosedLoopController m_controller;//    = m_motor.getClosedLoopController();


    //m_encoder is a seperate part in the motor.
    //this is why you need to make its own config.

    DoublePublisher encoder_publisher = NetworkTableInstance.getDefault().getDoubleTopic("L1/encoder1value").publish();//I'm guessing this makes a new section for L1

    ArmFeedforward m_feedforward = new ArmFeedforward(
        Constants.L1Constants.kL1kS, //volts 
        Constants.L1Constants.kL1kG, //volts,  test this using revclient 
        Constants.L1Constants.kL1kV, //volts * seconds / radians
        Constants.L1Constants.kL1kA  //volts * seconds ^ 2 / radians
        );//In case we need this if L1 needs to be more accurate, smooth

    
    public double m_targetPositionMode;

    public L1Subsystem(){
        m_motor   = new SparkMax(Constants.L1Constants.kL1Motor, MotorType.kBrushed);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller    = m_motor.getClosedLoopController();
        //1 motor subsytem spinning back and forth
        //Need to change the voltage for the gravity because the weight of the coral is not negligible, with setgains method.

        m_motorConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(360)
        .velocityConversionFactor(360);
        // .setSparkMaxDataPortConfig(); //i think this configures it to the encoder through the controller, but i'm not sure.

        m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //Maybe this is how you do it? 
        .pid(Constants.L1Constants.kL1Kp,Constants.L1Constants.kL1Ki,Constants.L1Constants.kL1Kd)//I don't understand the kSlot stuff. What do each of the slots represent. A: It represents pid settings that can be stored in each "slot".
        .outputRange(-0.3,0.3)//determines the speed limit. L1 TODO - this will likely need to be raised before tuning
        .positionWrappingEnabled(false)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-180.0, 180.0)
        .maxMotion
        .maxAcceleration(0)
        .maxVelocity(0) //I saw in the documentation that this is getting replaced with cruiseVelocity
        .allowedClosedLoopError(5);//TODO: Tune this L1 TODO - is this in degrees? It is, so 10 degrees for now.
        
        
        m_motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(Constants.L1Constants.kMaxCurrent);

        m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
        
        
    }

    public void spinForward(){
        //L1 TODO - you don't need this logic, reach goal takes care of getting to position
            m_motor.set(0.2);
            //reachGoal(Constants.L1Constants.extendedPosition);//Figure out which way is which
    }
    public void spinBackward(){
        //L1 TODO - you don't need this logic, reach goal takes care of getting to position
            m_motor.set(-0.1);
            //reachGoal(Constants.L1Constants.retractedPosition);
    }
    public void reachGoal(double goal){
        m_controller.setReference(goal,  //Docs says this is going to change to setSetpoint() in future versions.
                              ControlType.kPosition,//might mean to set the velocity to 0 (no velocity goal)
                              ClosedLoopSlot.kSlot0,
                              m_feedforward.calculate(convertDegtoRad(m_encoder.getPosition()), convertDegtoRad(m_encoder.getVelocity())));//armfeedforward
        
    }    ///reach goal 

    // L1 TODO - this function converts degrees to rads, not rot to radians. Rename please
    public double convertDegtoRad(double val){
        return val*Math.PI/180;
    }

    //Use this after we know L1 works
    //L1 TODO - copy in the feedforward to the regular reach goal. Update this to be a "hover" method which uses the ControlType.Voltage
    public void HoverL1(){
        m_controller.setReference(
            m_feedforward.calculate(convertDegtoRad(m_encoder.getPosition()), convertDegtoRad(m_encoder.getVelocity())),  //Docs says this is going to change to setSetpoint() in future versions.
                              ControlType.kVoltage);
    }
/**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
    public Command setGoal(double goal){
        return run(()-> reachGoal(goal));//converting runnable to command. 
    }

    public void stop(){
        m_motor.set(0);
    }
    

    @Override
    public void periodic(){
        // L1 TODO - don't these two publish the same thing? Can delete one of them
        encoder_publisher.set(m_encoder.getPosition());
        //if the elevator position is too low, automatically extend the l1 
        //or the velocity
    }
    

}