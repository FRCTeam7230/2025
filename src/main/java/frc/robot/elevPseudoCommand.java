
package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevPseudo;

public class elevPseudoCommand extends Command {
    elevPseudo elev = new elevPseudo();
    int phase = 0;
   /* motor1.raiseelevator();
    motor1.lowerelevator();
    motor1.stopmotor();
    tracker1.resetcount();
    tracker1.finddistance();
    tracker1.getDirection();*/
    //@Override
    public void execute(){
    //if (raisebutton.pressed()){
        if (phase==0){
            elev.raiseelevator();
            if (elev.finddistance()>100){
                phase+=1;
            }
        }
        if (phase==1){
            elev.lowerelevator();
            if (elev.finddistance()<80){
                phase+=1;
            }
        }
        if (phase==2){
            //Drive backwards.
        }
    }
    //@Override
    public boolean stops(){
        if (phase==2){
            return true;//Stop this command.
        } else {
            return false;
        }
    }
    
}