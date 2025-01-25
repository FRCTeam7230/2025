public class motor extends SubsystemBase {
    Spark elev = new Spark(1,MotorType.kBrushless);
    public void raiseelevator(){
        elev.set(2.000); //Full forward
    }
    public void lowerelevator(){
        elev.set(0.999);
    }
    public void stopmotor(){
        elev.stopMotor();
    }
}
public class tracker extends SubsystemBase{
    Encoder encode = new Encoder(1,2);
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
public class motormoving extends Command {
    motor motor1 = new motor();
    tracker tracker1 = new tracker();
    int phase = 0;
   /* motor1.raiseelevator();
    motor1.lowerelevator();
    motor1.stopmotor();
    tracker1.resetcount();
    tracker1.finddistance();
    tracker1.getDirection();*/
    @Override
    public void execute (){
    //if (raisebutton.pressed()){
        if (phase==0){
            motor1.raiseelevator();
            if (tracker1.finddistance()>100){
                phase+=1;
            }
        }
        if (phase==1){
            motor1.lowerelevator();
            if (tracker1.finddistance()<80){
                phase+=1;
            }
        }
        if (phase==2){
            //Drive backwards.
        }
    //}
    }
    public boolean stops (){
        if (phase==2){
            return true;//Stop this command.
        }
    }
}
public class elevator {
    public void Main(){
        Joystick joystick = new Joystick(0);
        JoystickButton button1 = new JoystickButton(joystick, 1);
        button1.pressed(new motormoving());
    }
}