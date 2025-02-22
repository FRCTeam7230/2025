package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;
public class AutoElevatorCommand extends Command {
    
    private ElevatorSubsystem m_elev;
    private double finGoal;
    public AutoElevatorCommand(ElevatorSubsystem elevator, double goal){
        m_elev = elevator;
        finGoal = goal;
        addRequirements(m_elev);
    }
    @Override
    public void initialize(){
        //Intializes Robot
    }
    @Override
    public void execute(){
        m_elev.reachGoal(finGoal);
    }
    @Override
    public void end(boolean interrupted){
        m_elev.stop();
    }
    @Override 
    public boolean isFinished(){
        if(Math.abs(m_elev.getHeight()-finGoal)<0.02){
            return true;
        } else {
            return false;
        }
    }   
}
