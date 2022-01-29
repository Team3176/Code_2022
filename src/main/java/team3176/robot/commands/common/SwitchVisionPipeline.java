package team3176.robot.commands.common;

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
        mSubsystem.SetVisionProcessing(true);
    }

    @Override
    public void execute(){
        mSubsystem.changePipeline((mSubsystem.getCurrentPipeline() == 1) ? 2 : 1);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
