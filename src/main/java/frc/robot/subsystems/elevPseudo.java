package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//might need to delete this file + the command
public class elevPseudo extends SubsystemBase {
    Spark elev = new Spark(1);
    Encoder encode = new Encoder(1,2);
    public void raiseelevator(){
        elev.set(2.000); //Full forward
    }
    public void lowerelevator(){
        elev.set(0.999);
    }
    public void stopmotor(){
        elev.stopMotor();
    }
    public void resetcount(){
        encode.reset();
    }
    public double finddistance(){
        return encode.getDistance();//Alternatively, use getposition();
    }
    public boolean finddirection(){
        return encode.getDirection();
    }
    
}

