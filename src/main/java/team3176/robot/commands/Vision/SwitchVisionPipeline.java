package team3176.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class SwitchVisionPipeline extends CommandBase {
    private final Vision m_Vision = Vision.getInstance();

    public SwitchVisionPipeline() {
        addRequirements(m_Vision);
    }

    @Override
    public void initialize(){
        m_Vision.setVisionProcessing(true);
    }

    @Override
    public void execute(){
        m_Vision.setActivePipeline((m_Vision.getCurrentPipeline() == 2) ? 0 : 2);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}