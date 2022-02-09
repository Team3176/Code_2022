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
        double result = mSubsystem.calculateDeltaX();
        System.out.println("I RAN!!!");
        addDistance(result);
    }

    private void addDistance(double newDistance){
        System.out.println("I RAN TOO!!!");
        for(int i = 0; i < 10; i++){
            if(distances[i] == 0){
                distances[i] = newDistance;
            }
        }
    }

    private boolean checkIfFinished(){
        System.out.println("I RAN THREE!!!");
        double sum = 0;
        for(double currValue : distances){
            if(currValue == 0){
                System.out.println("I CAME BACK FALSE!!!");
                return false;
            }
            sum += currValue;
        }
        averageDistance = sum / 10;
        System.out.println("I CAME BACK TRUE!!!");
        return true;
    }

    @Override
    public boolean isFinished(){
        if(checkIfFinished()){
            System.out.println("I FINISHED!!!");
            SmartDashboard.putNumber("Average Distance", averageDistance);
            return true;
        }
        return false;
    }
}
