package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;


import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.config.SmartMotionConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class AddAutoSubsystem extends SubsystemBase{
    //Now capable of storing many combined autos. Not sure if this will help for the interface.
    //We can always revert back to having just 1 in the robot container.
    ArrayList<PathPlannerAuto> pathList;

    public AddAutoSubsystem(ArrayList<PathPlannerAuto> path){
        pathList = path;
    }
    public void addPathToEnd(PathPlannerAuto path){//Adds path 
        pathList.add(path);//Adds path from the end
    }
    public void addPathUsingString(String path){//Adds path by taking in name of path.
        pathList.add(new PathPlannerAuto(path));
    }
    public void addPathToCertainIndex(int index,PathPlannerAuto path){//Adds path in a certain index.
        pathList.add(index,path);
    }
    public void deletePathByPath(PathPlannerAuto path){
        pathList.remove(path);//Removes by path name. So if path 1 wants to be removed, the first instance of it will be removed. Not reliable if there are multiple instances of this. 
    }
    public void deletePathByIndex(int index){
        pathList.remove(index);//Removes by index of array. Guaranteed to remove the path desired if interface can do it like this.
    }
    public void replacePath(int index, PathPlannerAuto path){//Replaces path
        pathList.remove(index);
        pathList.add(index,path);
    }
    public void displayPaths(){
        String[] array = new String[pathList.size()]; 
        int num = 0;
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){
            array[num] = i.next().getName();
            num+=1;
        }
        SmartDashboard.putStringArray("Path Names",array);//Can't see this in smardashboard
    }
    public void configurePathsAuto(String autoName){//Combines paths into a giant auto.
        SequentialCommandGroup newPathList = new SequentialCommandGroup();
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){//Loops through the entire array
            newPathList.addCommands(i.next());
        }
        SmartDashboard.putData(autoName, newPathList);
    }//This method will create a new SequentialCommandGroup based on the array by simply looping around it.
    public SequentialCommandGroup configurePaths(){//Combines paths into a giant auto.
        SequentialCommandGroup newPathList = new SequentialCommandGroup();
        for(Iterator <PathPlannerAuto> i = pathList.iterator(); i.hasNext();){//Loops through the entire array
            newPathList.addCommands(i.next());
        }
        return newPathList;
    }//This method will create a new SequentialCommandGroup based on the array by simply looping around it.
}
