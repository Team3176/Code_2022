package team3176.robot.commands.common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class CalculateTargetDistance extends CommandBase {

    private final Vision mSubsystem;    

    public CalculateTargetDistance(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        mSubsystem.calculateDistance();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
