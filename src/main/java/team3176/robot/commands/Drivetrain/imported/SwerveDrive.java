package team3176.robot.commands.Drivetrain.imported;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.SwerveSubsystemConstants;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem.driveMode;
import team3176.robot.subsystems.SwerveSubsystem.Gyro3176;

public class SwerveDrive extends CommandBase {
  private SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();

  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    addRequirements(SwerveSubsystem);
    addRequirements(m_CoordSys);
  }

  @Override
  public void initialize() {
    SwerveSubsystem.setDriveMode(driveMode.DRIVE);
    m_gyro.setSpinLockToOff();
  }

  @Override
  public void execute() {
    SwerveSubsystem.drive(forwardCommand.getAsDouble() * SwerveSubsystemConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, 
    strafeCommand.getAsDouble() * SwerveSubsystemConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, 
    spinCommand.getAsDouble()*10, m_CoordSys.getCurrentCoordType());
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}