package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class CalculateCameraTargetDistance extends CommandBase {

    private final Vision mSubsystem;
    private double result;

    public CalculateCameraTargetDistance(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        result = mSubsystem.calculateDeltaX();
        System.out.println("I RAN!!!");
        SmartDashboard.putNumber("Result", result);
    }

    @Override
    public boolean isFinished(){
        System.out.println("I FINISHED!!!");
        mSubsystem.averageMeasurements(result);
        return true;
    }
}