package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class elevPseudo extends SubsystemBase {
    //Credit: https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/MAXMotion/src/main/java/frc/robot/Robot.java
    SparkMax elev = new SparkMax(1,MotorType.kBrushless);
    RelativeEncoder encode = elev.getAlternateEncoder();
    
    public void raiseelevator(){
        elev.set(1.000); //Full forward
    }
    public void lowerelevator(){
        elev.set(-1);
    }
    public void stopmotor(){
        elev.stopMotor();
    }
    public void resetcount(){
        encode.setPosition(0.0);//Set # of rotations to 0.
    }
    public double finddistance(){
        return encode.getPosition();//Alternatively, use getposition();
    }
    public double findvelocity(){
        return encode.getVelocity();
    }

}

