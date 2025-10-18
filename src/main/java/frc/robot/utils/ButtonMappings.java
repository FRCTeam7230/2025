package frc.robot.utils;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ButtonMappings {
    public static Trigger button(GenericHID controller, int buttonNumber){
        if(controller instanceof Joystick){ //joystick
            if(buttonNumber == 15){ 
                return new Trigger(() -> ((Joystick) controller).getThrottle() < -0.70);
            } 
            if(buttonNumber == 30){
                return new Trigger(() -> ((Joystick) controller).getThrottle() > 0.70);
            }
        } else {
            //xboxcontoller
            if(buttonNumber == -2){
                return new Trigger(() -> controller.getRawAxis(/*change*/2) > 0.5);
            }
            if(buttonNumber == -3){
                return new Trigger(() -> controller.getRawAxis(/*change*/3) > 0.5);
            }
        }
        return buttonNumber%45==0 ? new POVButton(controller, buttonNumber) : new JoystickButton(controller, buttonNumber);
    }
}
