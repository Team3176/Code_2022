package team3176.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Vision;

public class SwitchVisionMode extends CommandBase {
    private final Vision mSubsystem;

    public SwitchVisionMode(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        mSubsystem.setVisionProcessing(!mSubsystem.getVisionProcessing());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
