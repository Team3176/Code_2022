package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class CalculateTargetDistance extends CommandBase {
    
    private final Vision mSubsystem;
    private double result;

    public CalculateTargetDistance(Vision subsystem) {
        mSubsystem = subsystem;
    }
    
    @Override
    public void execute(){
        mSubsystem.targetRecogControlLoop();
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
