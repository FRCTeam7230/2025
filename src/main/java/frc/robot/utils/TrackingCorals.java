package frc.robot.utils;
import java.util.ArrayList;

public class TrackingCorals {
    
    boolean[] corals = {false,false,false,false,false};//Perhaps make this global?
    double[] coralposx = {100,200,300,150,250,350};//Since coral is going through the reef, we can ignore the possible displacement.
    double[] coralposy = {200,300,100,300,200,100};
    ArrayList distances = new ArrayList<>();
    ArrayList distofcorals = new ArrayList<>();
    double robotposx = 0;
    double robotposy = 0;
    double robotposa = 0;
    int fieldofview = 0;
    
    public void main(){
    //See the corals and see which one is filled.
    //Use a vision system and figure out the location of the placed coral on the reef based on distance, color etc.
    //Using x and y positions of robot, approximate distance of coral.
    //I hope the camera can see this.
        for (int index = 0; index < corals.length; index++){
            double a = Math.atan2(coralposy[index]-robotposy,coralposx[index]-robotposx);
            if (a>robotposa-fieldofview/2&&a<robotposa+fieldofview/2){//Robot can only see these corals anyway
                double dist =Math.sqrt(
                    Math.pow(robotposx-coralposx[index],2)+
                    Math.pow(robotposy-coralposy[index],2)+
                    Math.pow(0,2)
                );//Called pythagorethem theorem, might need to consider the height of the reef too.
                distances.add(dist);//Distance in the simulated coral.
            } else {
                distances.add(-1);//Saving resources?
            }
        }
        distofcorals.add(10);//Vision system recongnizing corals and adding the distance from the camera for that.
        distofcorals.add(100);//Adding an if statement whether the coral is within the reef will be nice to prevent misunderstandings. Or finding how far up it is.
        //One major worry I have is what if the robot recognizes corals in a different level which causes it to register the wrong slot.
        for (int i = 0; i < corals.length; i++){
            double distOfSim = new Double(distances.get(i).toString());
            double distOfReal = new Double(distofcorals.get(i).toString());
            if (distOfSim>0&&corals[i]==false){//We never have to check the coral that is already in the reef because it can never get out. 
                for (int j = 0; j < distofcorals.size(); j++){
                    double simulatedCoralDistance = distOfSim;
                    double realCoralDistance = distOfReal;
                    if (Math.abs(simulatedCoralDistance-realCoralDistance)<1){
                        corals[i] = true;
                    }
                }
            }
        }//This feels inefficiet.
    }
}
//Afterwards, run the path based on available corals/
