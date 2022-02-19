package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class CalculateCameraTargetDistance extends CommandBase {

    private final Vision mSubsystem;
    private double[] distances;
    private double averageDistance;

    public CalculateCameraTargetDistance(Vision subsystem){
        mSubsystem = subsystem;
        distances = new double[10];
        averageDistance = 0;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        double result = mSubsystem.calculateDeltaXCam();
        addDistance(result);
    }

    private void addDistance(double newDistance){
        for(int i = 0; i < 10; i++){
            if(distances[i] == 0){
                distances[i] = newDistance;
            }
        }
    }

    private boolean checkIfFinished(){
        double sum = 0;
        for(double currValue : distances){
            if(currValue == 0){
                return false;
            }
            sum += currValue;
        }
        averageDistance = sum / 10;
        return true;
    }

    @Override
    public boolean isFinished(){
        if(checkIfFinished()){
            SmartDashboard.putNumber("Average Distance", averageDistance);
            return true;
        }
        return false;
    }
}
