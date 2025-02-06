package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExecuteAutonmous extends SubsystemBase {
    Timer time = new Timer();
    PathPlannerAuto a = new PathPlannerAuto("Example Auto");
    int phase = 0;
    PathPlannerAuto[] paths = {a,};//Array of paths stored for a sequence, potentially.
    //a.setCurrentTrajectory());
    public void Phase(PathPlannerAuto origPath,PathPlannerAuto newPath){
        origPath.end(false);//I don't know what this does. My guess is that the robot stops going on this path. Not sure if this is nessacery.
        phase+=1;
        newPath.execute();//I think this commands starts the new path? I'm unsure about this. No description in API. :/
    }
    public Command advancePhase(PathPlannerAuto origPath,PathPlannerAuto newPath){
        return run(() -> Phase(origPath,newPath));
    }
    public void IDKWhatToCallThis(int num){
        //a.initialize();
        if (paths[num].isFinished()){//If the command is finished or not? API says that the command executes end() naturally when it finishes. 
            if (num==0){
                paths[0].activePath("?").onTrue(advancePhase(new PathPlannerAuto(" "),paths[0]));//Just an idea here. I might need to 'end' a path that doesn't exist for the first path end.
            } else {
                paths[num].activePath("?").onTrue(advancePhase(paths[num-1],paths[num]));
            }
        }
    } 

    //@Override//Not sure why the code doesn't work with this.
    public void initialize(){
        for (int i = 0; i < paths.length; i++){
            paths[i].initialize();//Initializes all paths, not sure what to expect xD.
        }
    }

    @Override
    public void periodic(){
        paths[phase].execute();//Executes command or makes the robot go on this path, right?
    }
}