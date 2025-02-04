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

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motorController=new SparkMax(100, MotorType.kBrushless);
    private final RelativeEncoder encoder=motorController.getEncoder();
    private final double motorspeed = 1;
    public IntakeSubsystem(int motorID) {}  
    public void runMotor(){
        if (motorController.get()==0){//If the motor speed is 0 (when motor is not spinning)
            motorController.set(motorspeed);
        } else {
            motorController.set(0);
        }
        //motorController.set(motorspeed);
    }

    public void stopMotor() {
        motorController.set(0);
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