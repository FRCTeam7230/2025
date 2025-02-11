package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeRollerMotor = new SparkMax(Constants.IntakeConstants.intakeRollerMotorID, MotorType.kBrushless);
    //private final RelativeEncoder encoder=motorController.getEncoder();
    public IntakeSubsystem() {
    }  
    
    public void runMotor(){
        intakeRollerMotor.set(1);
        //motorController.set(motorspeed);
    }
    
    public void stopMotor() {
        intakeRollerMotor.stopMotor();
    }

    /*public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public double getEncoderSpeed() {
        return encoder.getVelocity();
    }

    public void resetEncoder() {
        encoder.reset();
    }*/
    @Override
    public void periodic()
    {
        //motorController.set(motorspeed);
    }
}