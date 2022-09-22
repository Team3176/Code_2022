package team3176.robot.commands.Drivetrain.imported;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.SwerveSubsystemConstants;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys.coordType;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem.driveMode;
import team3176.robot.subsystems.SwerveSubsystem.Gyro3176;

public class SwerveDriveTune extends CommandBase {
  private SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();

  private BooleanSupplier isFieldCentric;
  private BooleanSupplier isRobotCentric;

  public SwerveDriveTune() {
    //this.isFieldCentric = isFieldCentric;
    //this.isRobotCentric = isRobotCentric;
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

    if(m_CoordSys.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
      //SwerveSubsystem.setCoordType(coordType.FIELD_CENTRIC);
      m_CoordSys.setFieldCentricOffset();
    }
    //if(isRobotCentric.getAsBoolean()) {
    //  SwerveSubsystem.setCoordType(coordType.ROBOT_CENTRIC);
    //}
    SwerveSubsystem.drive(0,0,0);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}