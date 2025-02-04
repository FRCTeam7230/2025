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

public class intakesubsystem extends SubsystemBase {
    private final SparkMax motorController=new SparkMax(100, MotorType.kBrushless);
    private final Encoder encoder=motorController.getEncoder();


    public intakesubsystem(int motorID) {
        
    }

    public void setMotorSpeed(double speed) {
        motorController.set(speed);
    }

    public void stopMotor() {
        motorController.set(0);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public double getEncoderSpeed() {
        return encoder.getVelocity();
    }

    public void resetEncoder() {
        encoder.reset();
    }
}