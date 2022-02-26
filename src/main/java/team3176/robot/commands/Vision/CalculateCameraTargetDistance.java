package team3176.robot.commands.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Vision;

public class CalculateCameraTargetDistance extends CommandBase {

    private final Vision mSubsystem;    

    public CalculateCameraTargetDistance(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        double result = mSubsystem.calculateDeltaX();
        SmartDashboard.putNumber("Camera Distance", result);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
