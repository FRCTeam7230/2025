package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Iterator;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.config.SmartMotionConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddAutoSubsystem extends SubsystemBase{
    //TODO: Make the SmartDashboard add strings in the table.
    //Then, convert that to PathPlannerAuto making the string value the name of the auto.
    //Perhaps use this to allow users to select options: https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html
    ArrayList<PathPlannerAuto> pathList = new ArrayList<PathPlannerAuto>();
    
    public void addPathToEnd(PathPlannerAuto path){
        pathList.add(path);//Adds path from the end
        //SmartDashboard
    }
    public void addPathToCertainIndex(int index,PathPlannerAuto path){
        pathList.add(index,path);
    }
    public void deletePathByPath(PathPlannerAuto path){
        pathList.remove(path);//Removes by path name. So if path 1 wants to be removed, the first instance of it will be removed. Not reliable if there are multiple instances of this. 
    }
    public void deletePathByIndex(int index){
        pathList.remove(index);//Removes by index of array. Guaranteed to remove the path desired if interface can do it like this.
    }
    public void replacePath(int index, PathPlannerAuto path){
        pathList.remove(index);
        pathList.add(index,path);
    }
    public SequentialCommandGroup configurePaths(){
        SequentialCommandGroup newPathList = new SequentialCommandGroup();
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){//Loops through the entire array
            newPathList.addCommands(i.next());
        }
        return newPathList;
    }//This method will create a new SequentialCommandGroup based on the array by simply looping around it.
    public ArrayList<PathPlannerAuto> returnArray(){
        return pathList;
    }
}
