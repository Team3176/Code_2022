package team3176.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class SwitchVisionMode extends CommandBase {
    private final Vision m_Vision = Vision.getInstance();

    public SwitchVisionMode() {
        addRequirements(m_Vision);
    }

    @Override
    public void execute(){
        m_Vision.setVisionProcessing(!m_Vision.getVisionProcessing());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
