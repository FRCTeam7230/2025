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
    private final SparkMax intakeRollerMotor = new SparkMax(Constants.IntakeConstants.intakeRollerMotorID, MotorType.kBrushed);
  
    public IntakeSubsystem() {
    }  
    
    public void runIntakeRollerMotor(){
        intakeRollerMotor.set(-0.3);
    
    }
    
    public void stopIntakeRollerMotor() {
        intakeRollerMotor.stopMotor();
    }


    @Override
    public void periodic()
    {
        
    }
}