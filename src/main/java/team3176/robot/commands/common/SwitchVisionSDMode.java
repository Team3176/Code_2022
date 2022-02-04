package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class SwitchVisionSDMode extends CommandBase {
    private final Vision mSubsystem;

    public SwitchVisionSDMode(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        //mSubsystem.SetVisionProcessing(!mSubsystem.getVisionProcessing());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
