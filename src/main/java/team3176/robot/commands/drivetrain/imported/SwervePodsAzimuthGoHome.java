package team3176.robot.commands.drivetrain.imported;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;


public class SwervePodsAzimuthGoHome extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();

  public SwervePodsAzimuthGoHome()  {
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    m_Drivetrain.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {
    m_Drivetrain.sendPodsAzimuthToHome();
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setCurrentPodPosAsHome();
  }
}