package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;

public class TestVisionKinematics extends CommandBase {
    private final Vision mSubsystem;
    
    public TestVisionKinematics(Vision subsystem){
        mSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        double angle = SmartDashboard.getNumber("Angle", 45);
        double deltaX = SmartDashboard.getNumber("Test Delta X", 3);
        double deltaY = SmartDashboard.getNumber("Test Delta Y", 2);
        int angleIdx = 0;
        for(int i = 0; i < mSubsystem.initialAngle.length; i++){
            angleIdx = (angle == mSubsystem.initialAngle[i]) ? i : 0;
        }
        mSubsystem.TestVisionKinematics(angleIdx, deltaX, deltaY);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
