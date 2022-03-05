package team3176.robot.commands.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Vision;

public class CalculateTargetDistance extends CommandBase {
    
    private final Vision m_Vision = Vision.getInstance();
    private double result;

    public CalculateTargetDistance() {
        // addRequirements(m_Vision); //TODO: SEE IF IT IS NEEDED BECAUSE SAM DIDN'T HAVE
    }
    
    @Override
    public void execute(){
        m_Vision.targetRecogControlLoop();
    }
    
    @Override
    public boolean isFinished(){
        return true; //TODO: ADD LOGIC
    }
}
