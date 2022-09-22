package team3176.robot.commands.Drivetrain.imported;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem.driveMode;


public class SwervePodsAzimuthGoHome extends CommandBase {
  private SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();

  public SwervePodsAzimuthGoHome()  {
    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {
    SwerveSubsystem.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {
    SwerveSubsystem.sendPodsAzimuthToHome();
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.setCurrentPodPosAsHome();
  }
}