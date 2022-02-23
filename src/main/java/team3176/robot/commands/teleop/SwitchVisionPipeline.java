package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class SwitchVisionPipeline extends CommandBase {
    private final Vision mSubsystem;

    public SwitchVisionPipeline(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        mSubsystem.setVisionProcessing(true);
    }

    @Override
    public void execute(){
        mSubsystem.setActivePipeline((mSubsystem.getCurrentPipeline() == 2) ? 1 : 2);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
