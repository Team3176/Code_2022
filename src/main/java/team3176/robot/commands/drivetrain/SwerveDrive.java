package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.drivetrain.Gyro3176;

public class SwerveDrive extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();

  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    addRequirements(m_Drivetrain);
    addRequirements(m_CoordSys);
  }

  @Override
  public void initialize() {
    m_Drivetrain.setDriveMode(driveMode.DRIVE);
    m_gyro.setSpinLockToOff();
  }

  @Override
  public void execute() {
    m_Drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, 
    strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, 
    spinCommand.getAsDouble()*10, m_CoordSys.getCurrentCoordType());
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}