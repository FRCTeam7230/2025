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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class L1Subsystem extends SubsystemBase {
    private final SparkMax                  m_motor        = new SparkMax(Constants.L1Constants.kL1Motor, MotorType.kBrushless);
    private final AbsoluteEncoder           m_encoder       = m_motor.getAbsoluteEncoder();
    private final AbsoluteEncoderConfig     m_encoderConfig = new AbsoluteEncoderConfig();

    private final SparkMaxConfig            m_motorConfig        = new SparkMaxConfig();
    private final SparkClosedLoopController m_controller    = m_motor.getClosedLoopController();

    //m_encoder is a seperate part in the motor.
    //this is why you need to make its own config.

    DoublePublisher encoder_publisher = NetworkTableInstance.getDefault().getDoubleTopic("L1/encoder1value").publish();//I'm guessing this makes a new section for L1

    public L1Subsystem(){
        //1 motor subsytem spinning back and forth
        
        ArmFeedforward feed = new ArmFeedforward(0, 0, 0);//In case we need this if L1 needs to be more accurate, smooth
        //Need to change the voltage for the gravity because the weight of the coral is not negligible, with setgains method.
        
        m_encoderConfig   //maybe this isnt how you do it????
        .positionConversionFactor(360)
        .velocityConversionFactor(0);

        m_motorConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(0)
        .setSparkMaxDataPortConfig(); //i think this configures it to the encoder through the controller, but i'm not sure.

        m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //Maybe this is how you do it? 
        .pid(Constants.L1Constants.kL1Kp,Constants.L1Constants.kL1Ki,Constants.L1Constants.kL1Kd)//I don't understand the kSlot stuff. What do each of the slots represent. A: It represents pid settings that can be stored in each "slot".
        .outputRange(-1,1)//determines the speeeeed limit. 
        .maxMotion
        .maxAcceleration(0)
        .maxVelocity(0) //I saw in the documentation that this is getting replaced with cruiseVelocity
        .allowedClosedLoopError(0.1);//TODO: Tune this
        
        m_motorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_motorConfig.smartCurrentLimit(Constants.L1Constants.kMaxCurrent);

        m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
        
    }

    public void spinForward(){
        //if (m_encoder.getPosition()<Constants.L1Constants.extendedPosition){
            m_motor.set(0.1);
        //}
    }
    public void spinBackward(){
        //if (m_encoder.getPosition()>Constants.L1Constants.retractedPosition){
            m_motor.set(-0.1);
        //}
    }
    public void reachGoal(double goal){
        m_controller.setReference(goal,
                              ControlType.kPosition,//might mean to set the velocity to 0 (no velocity goal)
                              ClosedLoopSlot.kSlot0,
                             0);//armfeedforward
    }    ///reach goal 
    public void stop(){
        m_motor.set(0);
    }


    
    

    @Override
    public void periodic(){
        
        encoder_publisher.set(m_encoder.getPosition());
        SmartDashboard.putNumber("L1 Encoder Angle",m_encoder.getPosition());
        
    }
    // public void reachGoal(double goal)
    // {
    //     m_config.setReference(goal,
    //                             ControlType.kPosition,
    //                             ClosedLoopSlot.kSlot1,

    //                             );
    //     }

    // }

}