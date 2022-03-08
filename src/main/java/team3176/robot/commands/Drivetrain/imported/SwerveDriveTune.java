package team3176.robot.commands.Drivetrain.imported;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.drivetrain.Gyro3176;

public class SwerveDriveTune extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();

  private BooleanSupplier isFieldCentric;
  private BooleanSupplier isRobotCentric;

  public SwerveDriveTune() {
    //this.isFieldCentric = isFieldCentric;
    //this.isRobotCentric = isRobotCentric;
    addRequirements(drivetrain);
    addRequirements(m_CoordSys);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    m_gyro.setSpinLockToOff();
  }

  @Override
  public void execute() {

    if(m_CoordSys.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
      //drivetrain.setCoordType(coordType.FIELD_CENTRIC);
      m_CoordSys.setFieldCentricOffset();
    }
    //if(isRobotCentric.getAsBoolean()) {
    //  drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    //}
    drivetrain.drive(0,0,0);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}