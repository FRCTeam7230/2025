package frc.robot.commands;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
public class AutoElevatorCommand extends Command {
    private ElevatorSubsystem m_elev;
    private double finGoal;
    public AutoElevatorCommand(ElevatorSubsystem elevator, double goal){
        m_elev = elevator;
        finGoal = goal;
        addRequirements(m_elev);
    }
    public void reachGoal(double goal)
    {  
        finGoal = goal;
        m_elev.reachGoal(goal);
    }
    public void check(){
        Commands.print(Double.toString(m_elev.getHeight()));
        //SmartDashboard.putNumber("elevator height", m_elev.getHeight());
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
        if(Math.abs(m_elev.getHeight()-finGoal)<0.1){
            return true;
        } else {
            return false;
        }
    }   
}
